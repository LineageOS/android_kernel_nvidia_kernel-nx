subdir-ccflags-y += -Werror
obj-y += fuse/

obj-y += common.o
obj-y += flowctrl.o
obj-y += pmc.o
obj-y += pmc-r2p.o
obj-y += cvb.o

obj-$(CONFIG_ARM64)	             += pmc-smc-arm64.o
obj-$(CONFIG_TEGRA_DVFS)         += tegra-dvfs.o
obj-$(CONFIG_TEGRA_210_DVFS)     += tegra210-dvfs.o
