static int qtagpid_reset_stats(void)
{
	struct iface_stat *iface_entry;
	struct pid_stat *ps_entry;
	struct tag_stat *ts_entry;
	struct rb_node *node, *pid_node;

	spin_lock_bh(&iface_stat_list_lock);
	list_for_each_entry(iface_entry, &iface_stat_list, list) {
		spin_lock_bh(&iface_entry->tag_stat_list_lock);
		for (node = rb_first(&iface_entry->tag_stat_tree); node; node = rb_next(node)) {
			ts_entry = rb_entry(node, struct tag_stat, tn.node);
			for (pid_node = rb_first(&ts_entry->pid_stat_tree); pid_node; pid_node = rb_next(pid_node)) {
				ps_entry = rb_entry(pid_node, struct pid_stat, tn.node);
				//printk("ps_entry->tn.tag: %s\n", ps_entry->tn.tag);
				rb_erase(&ps_entry->tn.node, &ts_entry->pid_stat_tree);
			}
		}

		while (!list_empty(&iface_entry->pid_stat_list)) {
			ps_entry = list_entry(iface_entry->pid_stat_list.next, struct pid_stat, pslist);
			//printk("ps_entry->tn.tag: %s\n", ps_entry->tn.tag);
			ps_entry->index = 0;
			list_del(&ps_entry->pslist);
		}
		spin_unlock_bh(&iface_entry->tag_stat_list_lock);
	}
	spin_unlock_bh(&iface_stat_list_lock);

	return 1;
}

static void qtagpid_test_stats(void)
{
	struct iface_stat *iface_entry;
	struct tag_stat *ts_entry;
	struct rb_node *node, *pid_node;
	char comm[TASK_COMM_LEN];
	struct pid_stat *pid_stat_entry;
	int set;
	enum ifs_tx_rx direction;
	int proto;
	int bytes;
	int i;
	static int t = 0;

	spin_lock_bh(&iface_stat_list_lock);
	list_for_each_entry(iface_entry, &iface_stat_list, list) {
		spin_lock_bh(&iface_entry->tag_stat_list_lock);
		for (node = rb_first(&iface_entry->tag_stat_tree); node; node = rb_next(node)) {
			ts_entry = rb_entry(node, struct tag_stat, tn.node);
			for (pid_node = rb_first(&ts_entry->pid_stat_tree); pid_node; pid_node = rb_next(pid_node)) {
				for (i = 0; i < 100; i++) {
					spin_lock_bh(&pid_stat_tree_lock);
					spin_lock_bh(&ts_entry->pid_stat_list_lock);
					sprintf(comm, "test_%d_%d", t, i);
					set = 0;
					direction = i % 2;
					proto = i % 3;
					bytes = i;
					pid_stat_entry = pid_stat_tree_search(&ts_entry->pid_stat_tree, comm);
					if (!pid_stat_entry) {
						pid_stat_entry = create_pid_stat(ts_entry, comm, current->pid);
					}
					data_counters_update(&pid_stat_entry->counters, set, direction, proto, bytes);
					spin_unlock_bh(&ts_entry->pid_stat_list_lock);
					spin_unlock_bh(&pid_stat_tree_lock);
				}
			}
		}
		spin_unlock_bh(&iface_entry->tag_stat_list_lock);
	}
	spin_unlock_bh(&iface_stat_list_lock);
	t++;
}

static int qtagpid_set_split_uid_list(const char *input)
{
	char cmd;
	int argc;
	struct split_uid *new_uid;

	new_uid = kmalloc(sizeof(*new_uid), GFP_KERNEL);
	if (!new_uid) {
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&new_uid->list);
	argc = sscanf(input, "%c %d", &cmd, &new_uid->uid);
	CT_DEBUG("qtagpid: set_nearme_uid(%s): argc=%d cmd=%c new_uid=%d\n",
			input, argc, cmd, new_uid->uid);
	if (argc < 2) {
		return -EINVAL;
	}

	list_add_tail(&new_uid->list, &split_uid_list);

	return 0;
}

static ssize_t qtaguid_stats_pid_proc_write(struct file *file, const char __user *buffer, size_t count, loff_t *offp)
{
	if (!strncmp(buffer, "reset", count - 1))
		qtagpid_reset_stats();
	else if (!strncmp(buffer, "test", count - 1))
		qtagpid_test_stats();
	return count;
}

static void *qtaguid_stats_pid_proc_next(struct seq_file *m, void *v, loff_t *pos)
{
	struct proc_print_info *ppi = m->private;
	struct pid_stat *ps_entry;

	if (!v) {
		pr_err("qtaguid: %s(): unexpected v: NULL\n", __func__);
		return NULL;
	}

	(*pos)++;

	if (!ppi->iface_entry || unlikely(module_passive))
		return NULL;

	if (v == SEQ_START_TOKEN)
	    ps_entry = list_first_entry(&ppi->iface_entry->pid_stat_list, struct pid_stat, pslist);
	else
	    ps_entry = list_entry(((struct pid_stat *)v)->pslist.next, struct pid_stat, pslist);

	while (&ps_entry->pslist == &ppi->iface_entry->pid_stat_list) {
		qtaguid_stats_proc_next_iface_entry(ppi);
		if (!ppi->iface_entry)
			return NULL;
	    ps_entry = list_first_entry(&ppi->iface_entry->pid_stat_list, struct pid_stat, pslist);
	}

	ppi->ps_entry = ps_entry;
	ppi->tag_pos = *pos;
	ppi->tag_item_index = ppi->item_index;
	return ps_entry;
}

static void *qtaguid_stats_pid_proc_start(struct seq_file *m, loff_t *pos)
{
	struct proc_print_info *ppi = m->private;
	struct pid_stat *ps_entry = NULL;

	spin_lock_bh(&iface_stat_list_lock);

	if (*pos == 0) {
		ppi->item_index = 1;
		ppi->tag_pos = 0;
		if (list_empty(&iface_stat_list)) {
			ppi->iface_entry = NULL;
		} else {
			ppi->iface_entry = list_first_entry(&iface_stat_list,
							    struct iface_stat,
							    list);
			spin_lock_bh(&ppi->iface_entry->tag_stat_list_lock);
		}
		return SEQ_START_TOKEN;
	}
	if (!qtaguid_stats_proc_iface_stat_ptr_valid(ppi->iface_entry)) {
		if (ppi->iface_entry) {
			pr_err("qtaguid: %s(): iface_entry %p not found\n",
			       __func__, ppi->iface_entry);
			ppi->iface_entry = NULL;
		}
		return NULL;
	}

	spin_lock_bh(&ppi->iface_entry->tag_stat_list_lock);

	if (!ppi->tag_pos) {
		/* seq_read skipped first next call */
		ps_entry = SEQ_START_TOKEN;
	} else {
		//Haiping.Zhong@swdp.android.kernel, 2014/12/26, modify for netdiag /proc/net mtk ALPS01879491
		//ps_entry = list_entry(ppi->ps_entry->pslist.next, struct pid_stat, pslist);
		ps_entry = list_entry(&ppi->ps_entry->pslist, struct pid_stat, pslist);
		if (!ps_entry) {
			pr_info("qtaguid: %s(): tag_stat.tag 0x%llx not found. Abort.\n",
					__func__, ppi->tag);
			return NULL;
		}
	}

	if (*pos == ppi->tag_pos) { /* normal resume */
		ppi->item_index = ppi->tag_item_index;
	} else {
		/* seq_read skipped a next call */
		*pos = ppi->tag_pos;
		ps_entry = qtaguid_stats_pid_proc_next(m, ps_entry, pos);
	}

	return ps_entry;
}

static void pp_stats_pid_header(struct seq_file *m)
{
	seq_puts(m,
		 "idx iface acct_tag_hex uid_tag_int cnt_set "
		 "rx_bytes rx_packets "
		 "tx_bytes tx_packets "
		 "pid comm "
		 "rx_tcp_bytes rx_tcp_packets "
		 "rx_udp_bytes rx_udp_packets "
		 "rx_other_bytes rx_other_packets "
		 "tx_tcp_bytes tx_tcp_packets "
		 "tx_udp_bytes tx_udp_packets "
		 "tx_other_bytes tx_other_packets\n");
}

static int pp_stats_pid_line(struct seq_file *m, struct pid_stat *ps_entry,
			 int cnt_set)
{
	int ret;
	struct data_counters *cnts;
	uid_t stat_uid = get_uid_from_tag(ps_entry->uid);
	struct proc_print_info *ppi = m->private;
	/* Detailed tags are not available to everybody */
	if (get_atag_from_tag(ps_entry->uid) && !can_read_other_uid_stats(make_kuid(&init_user_ns,stat_uid))) {
		CT_DEBUG("qtaguid: stats line: "
			 "%s 0x%llx %u: insufficient priv "
			 "from pid=%u tgid=%u uid=%u stats.gid=%u\n",
			 ppi->iface_entry->ifname,
			 get_atag_from_tag(ps_entry->uid), stat_uid,
			 current->pid, current->tgid, from_kuid(&init_user_ns, current_fsuid()),
                         from_kgid(&init_user_ns,xt_qtaguid_stats_file->gid));
		return 0;
	}
	ppi->item_index++;
	cnts = &ps_entry->counters;
	ret = seq_printf(m, "%d %s 0x%llx %u %u "
		"%llu %llu "
		"%llu %llu "
		"%d %s "
		"%llu %llu "
		"%llu %llu "
		"%llu %llu "
		"%llu %llu "
		"%llu %llu "
		"%llu %llu %d\n",
		ppi->item_index,
		ppi->iface_entry->ifname,
		get_atag_from_tag(ps_entry->uid),
		stat_uid,
		cnt_set,
		dc_sum_bytes(cnts, cnt_set, IFS_RX),
		dc_sum_packets(cnts, cnt_set, IFS_RX),
		dc_sum_bytes(cnts, cnt_set, IFS_TX),
		dc_sum_packets(cnts, cnt_set, IFS_TX),
		ps_entry->pid, ps_entry->tn.tag,
		cnts->bpc[cnt_set][IFS_RX][IFS_TCP].bytes,
		cnts->bpc[cnt_set][IFS_RX][IFS_TCP].packets,
		cnts->bpc[cnt_set][IFS_RX][IFS_UDP].bytes,
		cnts->bpc[cnt_set][IFS_RX][IFS_UDP].packets,
		cnts->bpc[cnt_set][IFS_RX][IFS_PROTO_OTHER].bytes,
		cnts->bpc[cnt_set][IFS_RX][IFS_PROTO_OTHER].packets,
		cnts->bpc[cnt_set][IFS_TX][IFS_TCP].bytes,
		cnts->bpc[cnt_set][IFS_TX][IFS_TCP].packets,
		cnts->bpc[cnt_set][IFS_TX][IFS_UDP].bytes,
		cnts->bpc[cnt_set][IFS_TX][IFS_UDP].packets,
		cnts->bpc[cnt_set][IFS_TX][IFS_PROTO_OTHER].bytes,
		cnts->bpc[cnt_set][IFS_TX][IFS_PROTO_OTHER].packets,
		ps_entry->index);
	return ret ?: 1;
}

static bool pp_sets_pid(struct seq_file *m, struct pid_stat *ps_entry)
{
	int ret;
	int counter_set;
	for (counter_set = 0; counter_set < IFS_MAX_COUNTER_SETS;
	     counter_set++) {
		ret = pp_stats_pid_line(m, ps_entry, counter_set);
		if (ret < 0)
			return false;
	}
	return true;
}

/*
 * Procfs reader to get all tag stats using style "1)" as described in
 * fs/proc/generic.c
 * Groups all protocols tx/rx bytes.
 */
static int qtaguid_stats_pid_proc_show(struct seq_file *m, void *v)
{
	struct pid_stat *ps_entry = v;

	if (v == SEQ_START_TOKEN)
		pp_stats_pid_header(m);
	else
		pp_sets_pid(m, ps_entry);

	return 0;
}

static const struct seq_operations proc_qtaguid_stats_pid_seqops = {
	.start = qtaguid_stats_pid_proc_start,
	.next = qtaguid_stats_pid_proc_next,
	.stop = qtaguid_stats_proc_stop,
	.show = qtaguid_stats_pid_proc_show,
};

static int proc_qtaguid_stats_pid_open(struct inode *inode, struct file *file)
{
	return seq_open_private(file, &proc_qtaguid_stats_pid_seqops,
				sizeof(struct proc_print_info));
}

static const struct file_operations proc_qtaguid_stats_pid_fops = {
	.open		= proc_qtaguid_stats_pid_open,
	.read		= seq_read,
	.write		= qtaguid_stats_pid_proc_write,
	.llseek		= seq_lseek,
	.release	= seq_release_private,
};
