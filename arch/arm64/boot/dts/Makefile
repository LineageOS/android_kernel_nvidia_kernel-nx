dts-dirs += al
dts-dirs += altera
dts-dirs += amd
dts-dirs += amlogic
dts-dirs += apm
dts-dirs += arm
dts-dirs += broadcom
dts-dirs += cavium
dts-dirs += exynos
dts-dirs += freescale
dts-dirs += hisilicon
dts-dirs += marvell
dts-dirs += mediatek
dts-dirs += qcom
dts-dirs += renesas
dts-dirs += rockchip
dts-dirs += socionext
dts-dirs += sprd
dts-dirs += xilinx
dts-dirs += lg
dts-dirs += zte

subdir-y	:= $(dts-dirs)

dtstree		:= $(srctree)/$(src)

dtb-$(CONFIG_OF_ALL_DTBS) := $(patsubst $(dtstree)/%.dts,%.dtb, $(foreach d,$(dts-dirs), $(wildcard $(dtstree)/$(d)/*.dts)))
dtbo-$(CONFIG_OF_ALL_DTBS) := $(patsubst $(dtstree)/%.dts,%.dtbo, $(foreach d,$(dts-dirs), $(wildcard $(dtstree)/$(d)/*.dts)))

always		:= $(dtb-y)

targets += dtbs

# Enable DTB symbols
DTC_FLAGS += -@

DTC_FLAGS := -i $(tegra-dtstree)/soc/tegra/kernel-include
DTC_FLAGS += -i $(tegra-dtstree)/platform/tegra/common/kernel-dts
DTCCPP_FLAGS := -I$(tegra-dtstree)/soc/tegra/kernel-include
DTCCPP_FLAGS += -I$(tegra-dtstree)/platform/tegra/common/kernel-dts
DTCCPP_FLAGS += -DLINUX_VERSION=409

ifeq ($(CONFIG_ARCH_TEGRA_210_SOC),y)
DTC_FLAGS += -i $(tegra-dtstree)/soc/t210/kernel-dts
DTC_FLAGS += -i $(tegra-dtstree)/platform/t210/common/kernel-dts
DTC_FLAGS += -i $(tegra-dtstree)/platform/t210b01/common/kernel-dts
DTCCPP_FLAGS += -I$(tegra-dtstree)/soc/t210/kernel-dts
DTCCPP_FLAGS += -I$(tegra-dtstree)/platform/t210/common/kernel-dts
DTCCPP_FLAGS += -I$(tegra-dtstree)/platform/t210b01/common/kernel-dts
endif

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
DTC_FLAGS += -i $(tegra-dtstree)/soc/t18x/kernel-include
DTC_FLAGS += -i $(tegra-dtstree)/soc/t18x/kernel-dts
DTC_FLAGS += -i $(tegra-dtstree)/platform/t18x/common/kernel-dts
DTCCPP_FLAGS += -I$(tegra-dtstree)/soc/t18x/kernel-include
DTCCPP_FLAGS += -I$(tegra-dtstree)/soc/t18x/kernel-dts
DTCCPP_FLAGS += -I$(tegra-dtstree)/platform/t18x/common/kernel-dts
endif

ifeq ($(CONFIG_ARCH_TEGRA_19x_SOC),y)
DTC_FLAGS += -i $(tegra-dtstree)/soc/t19x/kernel-include
DTC_FLAGS += -i $(tegra-dtstree)/soc/t18x/kernel-include
DTC_FLAGS += -i $(tegra-dtstree)/soc/t19x/kernel-dts
DTC_FLAGS += -i $(tegra-dtstree)/platform/t19x/common/kernel-dts
DTC_FLAGS += -i $(tegra-dtstree)/platform/t19x/automotive/kernel-dts/common
DTCCPP_FLAGS += -I$(tegra-dtstree)/soc/t19x/kernel-include
DTCCPP_FLAGS += -I$(tegra-dtstree)/soc/t18x/kernel-include
DTCCPP_FLAGS += -I$(tegra-dtstree)/soc/t19x/kernel-dts
DTCCPP_FLAGS += -I$(tegra-dtstree)/platform/t19x/common/kernel-dts
DTCCPP_FLAGS += -I$(tegra-dtstree)/platform/t19x/automotive/kernel-dts/common
endif

ifeq ($(CONFIG_ANDROID),y)
DTCCPP_FLAGS += -DCONFIG_ANDROID
endif

dtb-y :=
dtbo-y :=
dts_makefile=$(foreach d,$(wildcard $1*), $(call dts_makefile,$(d)/,$(2)) $(if $(findstring Makefile,$(d)),$(d)))
dts_mfiles = $(call dts_makefile, $(tegra-dtstree)/platform/, Makefile)
ifneq ($(dts_mfiles),)
dts-include :=
include $(dts_mfiles)
ifneq ($(dts-include),)
DTC_FLAGS += $(addprefix -i $(tegra-dtstree)/,$(dts-include))
DTCCPP_FLAGS += $(addprefix -I$(tegra-dtstree)/,$(dts-include))
endif
endif

DTB_OBJS := $(addprefix $(obj)/,$(dtb-y))
DTBO_OBJS := $(addprefix $(obj)/,$(dtbo-y))
targets += $(dtb-y)

define _define_dtb_rule
$(obj)/$(1): $(src)/$(addprefix $(tegra-rel-dtstree)/hardware/nvidia/,$(patsubst %.dtb,%.dts,$(1))) FORCE
endef

$(foreach _dtb, $(dtb-y), $(eval $(call _define_dtb_rule,$(_dtb))))

$(DTB_OBJS):
	$(call if_changed_dep,dtc)

define _define_dtbo_rule
$(obj)/$(1): $(src)/$(addprefix $(tegra-rel-dtstree)/hardware/nvidia/,$(patsubst %.dtbo,%.dts,$(1))) FORCE
endef

$(foreach _dtbo, $(dtbo-y), $(eval $(call _define_dtbo_rule,$(_dtbo))))

$(DTBO_OBJS):
	$(call if_changed_dep,dtc)

dtbs: $(DTB_OBJS) $(DTBO_OBJS) FORCE
ifneq ($(DTB_OBJS),)
	cp $(DTB_OBJS) $(DTBO_OBJS) $(obj)/
endif

clean-files := dts/*.dtb *.dtb *.dtbo
clean-dirs := dts/_ddot_
