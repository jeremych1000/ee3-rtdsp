n=10;
Rp=0.4;
Rs=23;
fs=8000;
corner_f=[180 450]/(fs/2);
[b,a]=ellip(n,Rp,Rs,corner_f);
freqz(b,a);
fid=fopen('../RTDSPlab/lab5/fir_coef.txt','w');
fprintf(fid,'double b[]={');
fprintf(fid,'%.10e,\t',b);
fprintf(fid,'};\n');
fprintf(fid,'double a[]={');
fprintf(fid,'%.10e,\t',a);
fprintf(fid,'};\n');
fprintf(fid,'#define N %d',length(a)-1);
fclose(fid);
figure;
zplane(b, a);
