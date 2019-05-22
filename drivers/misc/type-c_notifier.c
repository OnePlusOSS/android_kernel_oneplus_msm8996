#include <linux/notifier.h>
#include <linux/export.h>
#include <linux/type-c_notifier.h>

static BLOCKING_NOTIFIER_HEAD(bc_notifier_list);

int bc_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&bc_notifier_list, nb);
}
EXPORT_SYMBOL(bc_register_client);

int bc_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&bc_notifier_list, nb);
}
EXPORT_SYMBOL(bc_unregister_client);

int bc_notifier_call_chain(unsigned long val)
{
	return blocking_notifier_call_chain(&bc_notifier_list, val, NULL);
}
EXPORT_SYMBOL_GPL(bc_notifier_call_chain);
