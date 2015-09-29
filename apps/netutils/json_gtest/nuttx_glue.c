int *get_errno_ptr(void)
{
  static int err = -1;
  return &err;
}
void up_assert(char *s)
{
  extern void exit(int);
  exit(1);
}
