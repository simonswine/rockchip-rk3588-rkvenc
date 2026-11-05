# Build against the configured kernel matching the target system.
KDIR ?= /lib/modules/$(shell uname -r)/build
MODULE_DIR := $(CURDIR)

.PHONY: all modules modules_install clean test
all: modules
modules:
	$(MAKE) -C $(KDIR) M=$(MODULE_DIR) modules
modules_install:
	$(MAKE) -C $(KDIR) M=$(MODULE_DIR) modules_install
clean:
	$(MAKE) -C $(KDIR) M=$(MODULE_DIR) clean
test:
	python3 check_driver.py
