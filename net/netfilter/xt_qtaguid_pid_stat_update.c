static int pid_compare(char *t1, char *t2)
{
	return strcmp(t1, t2);
}

static void pid_node_tree_insert(struct pid_node *data, struct rb_root *root)
{
	struct rb_node **new = &(root->rb_node), *parent = NULL;

	/* Figure out where to put new node */
	while (*new) {
		struct pid_node *this = rb_entry(*new, struct pid_node,
				node);
		int result = pid_compare(data->tag, this->tag);
		parent = *new;
		if (result < 0)
			new = &((*new)->rb_left);
		else if (result > 0)
			new = &((*new)->rb_right);
		else
			BUG();
	}

	/* Add new node and rebalance tree. */
	rb_link_node(&data->node, parent, new);
	rb_insert_color(&data->node, root);
}

static void pid_stat_tree_insert(struct pid_stat *data, struct rb_root *root)
{
	pid_node_tree_insert(&data->tn, root);
}

static struct pid_node *pid_node_tree_search(struct rb_root *root, char *tag)
{
	struct rb_node *node = root->rb_node;

	while (node) {
		struct pid_node *data = rb_entry(node, struct pid_node, node);
		int result;
		result = pid_compare(tag, data->tag);
		if (result < 0)
			node = node->rb_left;
		else if (result > 0)
			node = node->rb_right;
		else
			return data;
	}
	return NULL;
}

static struct pid_stat *pid_stat_tree_search(struct rb_root *root, char *tag)
{
	struct pid_node *node = pid_node_tree_search(root, tag);
	if (!node)
		return NULL;
	return rb_entry(&node->node, struct pid_stat, tn.node);
}

static struct pid_stat *create_pid_stat(struct tag_stat *tag_entry, char *comm, pid_t pid)
{
	struct pid_stat *new_pid_stat_entry = NULL;
	static int entry_index = 0; /* just for test */

	new_pid_stat_entry = kzalloc(sizeof(*new_pid_stat_entry), GFP_ATOMIC);
	if (!new_pid_stat_entry) {
		pr_err("qtaguid: create_pid_stat: tag stat alloc failed\n");
		goto done;
	}
	new_pid_stat_entry->pid = pid;
	new_pid_stat_entry->uid = tag_entry->tn.tag;
	new_pid_stat_entry->index = entry_index++; /* just for test */
	strncpy(new_pid_stat_entry->tn.tag, comm, TASK_COMM_LEN);

	pid_stat_tree_insert(new_pid_stat_entry, &tag_entry->pid_stat_tree);
	list_add_tail(&new_pid_stat_entry->pslist, &tag_entry->iface_stat->pid_stat_list);
done:
	return new_pid_stat_entry;
}

//caller must hold lock:
static void pid_stat_update(struct tag_stat *tag_entry, int set,
		    enum ifs_tx_rx direction, int proto, int bytes,
		    char *task_comm, pid_t task_pid)
{
	struct pid_stat *pid_stat_entry;

	if (task_comm != NULL) {
		spin_lock_bh(&pid_stat_tree_lock);
		spin_lock_bh(&tag_entry->pid_stat_list_lock);
		pid_stat_entry = pid_stat_tree_search(&tag_entry->pid_stat_tree, task_comm);
		if (!pid_stat_entry) {
			//printk("pid_stat_update create:%s\n", task_comm);
			pid_stat_entry = create_pid_stat(tag_entry, task_comm, task_pid);
		}
		data_counters_update(&pid_stat_entry->counters, set, direction, proto, bytes);
		spin_unlock_bh(&tag_entry->pid_stat_list_lock);
		spin_unlock_bh(&pid_stat_tree_lock);
	} else {
		//pr_err("pid_stat_update task_comm is NULL\n");
	}
}
