This programm can be used to simulate packet loss on RTP files. The C random number 
generator is used for selecting the packets. The random number generator is not 
initialized befur usage. Therefore subsequent runs of the tool will create identical 
loss pattern.

usage:

  rtp_loss.exe infile outfile losspercent <keep_leading_packets>


<keep_leading_packets> is an optinal parameter that specifies the number of RTP
packets that are kept at the beginning of the file. This can be useful if the loss
of parameter sets or IDR pictures shall be avoided.
