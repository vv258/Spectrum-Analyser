delete(instrfind)


s = serial('COM6');
set(s,'BaudRate',19200,'Timeout',10);

freq=[376,458,547,642,745,856,975,1103,1241,1389,1549,1721,1906,2105,2320,2551,2800,3067,3355,3666];


fopen(s);

while true
sound = fscanf(s,'%s');
c=textscan(sound,'%d','Delimiter',',');

bar(c{1});
set(gca, 'XTick', 1:20,'XTickLabel',freq);
ylim([0,256])
xlabel('Mel Frequency \rightarrow');
ylabel('Power \rightarrow');
title('Real-time sound plot');
drawnow
end

fclose(s);
