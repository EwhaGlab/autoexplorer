% mp planning time analysis

filepath = '/home/hankm/results/autoexploration/numthreads_vs_timing';
gptime_per_thread = zeros(16,1);
stdata = struct ;

for ntidx=1:5:16
    timedata = [];
    cnt = 1;
    for nround=1 %:8
        filename = sprintf('%s/planning_time_%d_%d.txt',filepath,ntidx, nround);
        mydata = importdata(filename) ;
        %cnt = 1;
        for idx=1:length(mydata)
            dat = str2num(mydata{idx}) ;
            if( length(dat) == 3 )
               timedata(cnt,:) = dat ; 
               cnt = cnt + 1;
            end
        end
    end
    stdata(ntidx).X = timedata ;
    c = polyfit( timedata(:,2), timedata(:,3), 1 ) ;
    stdata(ntidx).c = c;
    stdata(ntidx).fitest = polyval(c, timedata(:,2) ) ;
    
end

figure(1); clf; hold on;
plot(stdata(1).X(:,2), stdata(1).fitest, 'r-' );
plot(stdata(6).X(:,2), stdata(6).fitest, 'g-' );
plot(stdata(11).X(:,2), stdata(11).fitest, 'b-' );
plot(stdata(16).X(:,2), stdata(16).fitest, 'c-' );
xlabel('num fpts to process')
ylabel('runtime (ms)')
title('mp BB performance w/o init heuristic')
legend('t1', 't6', 't12', '16')
grid on






