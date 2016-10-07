clear;

%myDir = pwd
myDir = uigetdir; %gets directory
myFiles = dir(fullfile(myDir,'*.wav')); %gets all wav files in struct

for k = 1:length(myFiles)
    mysave(k).name = strrep(myFiles(k).name, '.wav', '.png');
    baseFileName = myFiles(k).name;
    filename = fullfile(myDir, baseFileName);
    fprintf(1, 'Now reading %s\n', filename);

    %file='car1_infilt(0)_outfilt(0)_alpha(5)_gainVar(1)';
    %type = '.wav';
    %type2 = '.png';
    %filename = strcat(file, type);

    Fs=8000;
    FFTLEN=256;
    OVERSAMP=4;
    noverlap=(FFTLEN/OVERSAMP)*(OVERSAMP-1);
    [y,Fs] = audioread(filename);
    window=hann(FFTLEN);

    spectrogram(y(:,1),FFTLEN,noverlap,FFTLEN,Fs,'yaxis');
    colormap(jet);
    fprintf(1, 'Now saving %s\n', filename);
    saveas(gcf, mysave(k).name);
    fprintf('Progress = %d/%d, %d%%\n', k, length(myFiles), round(100*k/length(myFiles)));
    close(gcf);
end

fprintf('Done');