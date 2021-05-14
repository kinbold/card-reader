#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Cwiegand-reader,input");
MODULE_ALIAS("of:N*T*Cwiegand-reader,inputC*");
MODULE_ALIAS("of:N*T*Caba-reader,input");
MODULE_ALIAS("of:N*T*Caba-reader,inputC*");
MODULE_ALIAS("of:N*T*Cem125,input");
MODULE_ALIAS("of:N*T*Cem125,inputC*");

MODULE_INFO(srcversion, "E19ED2444CCF5E8FD950863");
