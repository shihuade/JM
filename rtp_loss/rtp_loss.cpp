// rtp_loss.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#ifdef WIN32
#include <Winsock2.h>
#else
#include <netinet/in.h>
#endif

void print_usage(char *argv [])
{
  printf ("This tool allows dropping RTP packets from the given input file.");
  printf ("Note: the input file needs to be a H.264/AVC RTP dump file (lencod: OutFileMode=1)");
  printf ("Usage: %s input_file output_file loss_percent <keep_leading_packets>\n", argv[0]);
  exit (-1);
}

int keep_packet(int loss_percent)
{
  int rnd;
  if (loss_percent>100)
    return 1;
  if (loss_percent<=0)
    return 0;
  
  rnd = int (100 * (((float) rand()) / RAND_MAX));

  return (rnd >= loss_percent );
}

int main(int argc, char* argv[])
{
  unsigned int bufsize, pacno=0;
  unsigned char buf[65000];
  int i, intime;
  FILE *fr;                                // file for reading
  FILE *fw;                                // file for writing


  if ((argc != 4) && (argc != 5))
  {
    print_usage (argv);
  }

  if (NULL == (fr = fopen (argv[1], "rb")))
  {
    printf ("%s: cannot open H.264 packet file %s for reading\n", argv[0], argv[1]);
    return -2;
  }

  if (NULL == (fw = fopen (argv[2], "wb")))
  {
    printf ("%s: cannot open H.264 packet file %s for reading\n", argv[0], argv[1]);
    fclose (fr);
    return -2;
  }


  if (argc==5)
  {
    for (i=0; i< atoi (argv[4]); i++)
    {
      if (4 != fread (&bufsize, 1, 4, fr))
        return 0;
      if (4 != fread (&intime, 1, 4, fr))
      {
        printf ("Panic, cannot read timestamp (is this a valid RTP dump file?)\n");
        return -1;
      }
      if (bufsize != fread (buf, 1, bufsize, fr))
      {
        printf ("Cannot read packet with indicated length (is this a valid RTP dump file?), exit\n");
        return -3;
      }

      if (4 != fwrite (&bufsize, 1, 4, fw))
      {
        printf ("Problems while writing buffer size, exit\n");
        return -1;
      }
      if (4 != fwrite (&intime, 1, 4, fw))
      {
        printf ("Problems while writing timestamp, exit\n");
        return -1;
      }
      if (bufsize != fwrite (buf, 1, bufsize, fw))
      {
        printf ("Problems while writing buffer, exit\n");
        return -3;
      }
      pacno++;
    }

  }

  for (;;) 
  {
    if (4 != fread (&bufsize, 1, 4, fr))
      return 0;
    if (4 != fread (&intime, 1, 4, fr))
    {
      printf ("Panic, cannot read timestamp (is this a valid RTP dump file?)\n");
      return -1;
    }
    if (bufsize != fread (buf, 1, bufsize, fr))
    {
      printf ("Cannot read packet with indicated length (is this a valid RTP dump file?)\n");
      return -3;
    }
    if (keep_packet(atoi (argv[3])))
    {
      if (4 != fwrite (&bufsize, 1, 4, fw))
      {
        printf ("Problems while writing buffer size, exit\n");
        return -1;
      }
      if (4 != fwrite (&intime, 1, 4, fw))
      {
        printf ("Problems while writing timestamp, exit\n");
        return -1;
      }
      if (bufsize != fwrite (buf, 1, bufsize, fw))
      {
        printf ("Problems while writing buffer, exit\n");
        return -3;
      }
    }
    else
    {
      printf ("lost packet #%d\n", pacno);
    }
    pacno++;

  }
}
