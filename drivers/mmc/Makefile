#
# Makefile for the kernel mmc device drivers.
#

subdir-ccflags-y += -Werror
subdir-ccflags-$(CONFIG_MMC_DEBUG) := -DDEBUG

obj-$(CONFIG_MMC)		+= core/
obj-$(CONFIG_MMC)		+= card/
obj-$(subst m,y,$(CONFIG_MMC))	+= host/
obj-$(CONFIG_SLT_DEVINFO)		+= sltdevinfo/
