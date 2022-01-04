% mp planning time analysis

filepath = '/home/hankm/results/autoexploration/numthreads_vs_timing_bb';
gptime_per_thread = zeros(16,1);
stdata = struct ;

for ntidx=[1 4 8 12 16]
    timedata = [];
    cnt = 1;
    for nround=1 %:8
        filename = sprintf('%s/planning_time_%d_%d.txt',filepath,ntidx, nround);
        mydata = importdata(filename) ;
        %cnt = 1;
        for idx=1:length(mydata)
            dat = str2num(mydata{idx}) ;
            if( length(dat) == 3 )
%             dat = mydata(idx,:)
%             if( sum(isnan(dat)>0 ) )
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

figure(2); clf; hold on;
plot(stdata(1).X(:,2), stdata(1).fitest, 'r-<' );
plot(stdata(4).X(:,2), stdata(4).fitest, 'm-o' );
plot(stdata(8).X(:,2), stdata(8).fitest, 'g-s' );
plot(stdata(12).X(:,2), stdata(12).fitest, 'c-' );
plot(stdata(16).X(:,2), stdata(16).fitest, 'b-' );
xlabel('num fpts to process')
ylabel('runtime (ms)')
title('mp BB performance')
legend('t1', 't4', 't8', 't12', '16')
grid on






