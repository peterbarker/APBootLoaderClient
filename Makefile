all: test-bootloader

test-bootloader: test-bootloader.c bootloader_client.c slurp.c
