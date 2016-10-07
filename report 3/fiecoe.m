F=[375,450,1600,1750];
Fs=8000;
rp=0.4;
A=[0 1 0];
DEV=[10^(-48/20) (10^(rp/20)-1)/(10^(rp/20)+1) 10^(-48/20)];
[N,Fo,Ao,W]=firpmord(F,A,DEV,Fs);
N=N;
B=firpm(N,Fo,Ao,W);
freqz(B,1,4096,Fs);
fid=fopen('fir_coef.txt','w');
fprintf(fid,'double b[]={');
fprintf(fid,'%.10e,\t',B);
fprintf(fid,'};\n');
%fprintf(fid,'#define N %d',length(B));
fclose(fid);