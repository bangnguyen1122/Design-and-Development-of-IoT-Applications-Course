CONTIKI_PROJECT = tree_routing_bnn_prr_datasend
TARGET=sky
all: $(CONTIKI_PROJECT)

CFLAGS = -DWITH_UIP=1

CONTIKI_WITH_RIME = 1

# DEFINES=NETSTACK_CONF_RDC=contikimac_driver,NETSTACK_CONF_MAC=csma_driver
# DEFINES=NETSTACK_CONF_RDC=xmac_driver,NETSTACK_CONF_MAC=csma_driver
DEFINES=NETSTACK_CONF_RDC=nullrdc_driver,NETSTACK_CONF_MAC=csma_driver

CFLAGS += -std=c99

CONTIKI = ../../../..
include $(CONTIKI)/Makefile.include
