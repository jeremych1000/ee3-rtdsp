RC=10^-3;
sys=zpk([],[-1/RC],1/RC);
[num,den]=zp2tf([],[-1/RC],1/RC);
[zd,pd] = bilinear(num,den,8000); 
sys2=filt(zd,pd,1/8000);
bode_diag=bodeplot(sys2);
setoptions(bode_diag,'FreqUnits','Hz');
axis([1,3800,-90 ,0]);