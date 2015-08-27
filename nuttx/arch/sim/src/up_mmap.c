#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/mman.h>

void *
up_mmap (const char * const filename, size_t *len)
{
  void *addr = NULL;
  int fd;
  struct stat sb;

  fd = open (filename, O_RDONLY);
  if (fd == -1)
    {
      perror ("open");
      return NULL;
    }

  if (fstat (fd, &sb) == -1)
    {
      perror ("fstat");
      goto out;
    }

  addr = mmap (NULL, sb.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
  if (addr == NULL)
    {
      perror ("mmap");
      goto out;
    }

  *len = sb.st_size;

out:

  close(fd);

  return addr;
}
