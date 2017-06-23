#include "slurp.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

/*
 * slurp_file - read a file into memory
 */
int slurp_file(const char *filepath, uint8_t **fw)
{
    struct stat stat_buf;
    off_t bytes_read = -1;

    if (stat(filepath, &stat_buf) == -1) {
	fprintf(stderr, "Failed to stat (%s): %s\n", filepath, strerror(errno));
	goto ERR;
    }

    fprintf(stderr, "Loading (%s) of size %lu\n", filepath, stat_buf.st_size);
    int fw_fd = open(filepath, O_RDONLY);
    if (fw_fd == -1) {
	fprintf(stderr, "Failed to open (%s): %s\n", filepath, strerror(errno));
	goto ERR;
    }

    const off_t _size = stat_buf.st_size + (stat_buf.st_size%4);
    uint8_t *_fw = (uint8_t*)malloc(_size);
    if (_fw == NULL) {
	fprintf(stderr, "Failed to allocate (%lu) bytes\n", stat_buf.st_size);
	goto ERR_FH;
    }

    memset(_fw, '\xff', _size);

    bytes_read = 0;
    while (bytes_read < stat_buf.st_size) {
	const ssize_t n = read(fw_fd, &_fw[bytes_read], stat_buf.st_size-bytes_read);
	if (n == -1) {
	    fprintf(stderr, "Failed to read from (%s): %s\n", filepath, strerror(errno));
            bytes_read = -1;
	    goto ERR_FH;
	}
	if (n == 0) {
	    fprintf(stderr, "EOF reading from (%s): %s\n", filepath, strerror(errno));
            bytes_read = -1;
	    goto ERR_FH;
	}
	bytes_read += n;
    }

    *fw = _fw;

 ERR_FH:
    close(fw_fd);
 ERR:
    return bytes_read;
}

