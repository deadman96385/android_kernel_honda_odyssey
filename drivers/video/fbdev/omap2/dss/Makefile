obj-$(CONFIG_OMAP2_DSS_INIT) += omapdss-boot-init.o
obj-$(CONFIG_OMAP2_DSS) += omapdss.o
# Core DSS files
omapdss-y := core.o dss.o dss_dpll.o dss_features.o dispc.o dispc_coefs.o \
	display.o output.o dss-of.o
# DSS compat layer files
ifeq ($(SPECTRUM_DISPLAY),true)
     ccflags-y += -DSPECTRUM_DISPLAY
endif
omapdss-y += manager.o manager-sysfs.o overlay.o overlay-sysfs.o apply.o \
	dispc-compat.o display-sysfs.o
omapdss-$(CONFIG_OMAP2_DSS_DPI) += dpi.o
omapdss-$(CONFIG_OMAP2_CDCE913_CLOCK) += cdce913_i2c_client.o
omapdss-$(CONFIG_OMAP2_DSS_RFBI) += rfbi.o
omapdss-$(CONFIG_OMAP2_DSS_VENC) += venc.o
omapdss-$(CONFIG_OMAP2_DSS_SDI) += sdi.o
omapdss-$(CONFIG_OMAP2_DSS_DSI) += dsi.o pll.o
omapdss-$(CONFIG_OMAP2_DSS_HDMI_COMMON) += hdmi_common.o hdmi_wp.o pll.o \
	hdmi_phy.o
omapdss-$(CONFIG_OMAP4_DSS_HDMI) += hdmi4.o hdmi4_core.o
omapdss-$(CONFIG_OMAP5_DSS_HDMI) += hdmi5.o hdmi5_core.o
ccflags-$(CONFIG_OMAP2_DSS_DEBUG) += -DDEBUG
