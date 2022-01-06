% bb vs vanilla

vafilepath = '/home/hankm/results/autoexploration/numthreads_vs_timing_vanilla'
bbfilepath = '/home/hankm/results/autoexploration/numthreads_vs_timing_bb';
gptime_per_thread = zeros(16,1);
stdata = struct ;

for ntidx=[1 4 8 12 16]
    vatimedata = [];
    cnt = 1;
    for nround=1 %:8
        vafilename = sprintf('%s/planning_time_%d_%d.txt',vafilepath,ntidx, nround);
        vadata = importdata(vafilename) ;
        %cnt = 1;
        for idx=1:length(vadata)
%            dat = str2num(vadata{idx}) ;
%            if( length(dat) == 3 )
             dat = vadata(idx,:) ;
             if( sum(isnan(dat)>0 ) )
               vatimedata(cnt,:) = dat ; 
               cnt = cnt + 1;
            end
        end
    end
    stdata(ntidx).X_va = vatimedata ;
    c = polyfit( vatimedata(:,2), vatimedata(:,3), 1 ) ;
    stdata(ntidx).c_va = c;
    stdata(ntidx).fitest_va = polyval(c, vatimedata(:,2) ) ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Process BB data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    bbtimedata = [];
    cnt = 1;
    for nround=1:12
        bbfilename = sprintf('%s/planning_time_%d_%d.txt', bbfilepath, ntidx, nround);
        bbdata = importdata(bbfilename) ;
        for idx=1:length(bbdata)
            dat = str2num(bbdata{idx}) ;
            if( length(dat) == 3 )
               bbtimedata(cnt,:) = dat ; 
               cnt = cnt + 1;
            end
        end
    end
    stdata(ntidx).X_bb = bbtimedata ;
    c = polyfit( bbtimedata(:,2), bbtimedata(:,3), 1 ) ;
    stdata(ntidx).c_bb = c ;
    stdata(ntidx).fitest_bb = polyval(c, bbtimedata(:,2) ) ;
   
end


% draw res

figure(1); clf; hold on
plot(stdata(1).X_bb(:,2), stdata(1).X_bb(:,3), 'b<') ;
plot(stdata(8).X_bb(:,2), stdata(8).X_bb(:,3), 'gs');
plot(stdata(1).X_bb(:,2), stdata(1).fitest_bb, 'b--', 'color',[0.5 0 1] );
plot(stdata(8).X_bb(:,2), stdata(8).fitest_bb, 'g--', 'color', [0 1 1] );
grid on

xlabel('num of frontier points to process')
ylabel('runtime (ms)')
title('line fitting results on the runtime experiments')
legend('t1(bb)', 't8(bb)','line fit on t1(bb)', 'line fit on t8(bb)', 'Location', 'NW')
grid on




figure(2); clf; hold on;
plot(stdata(1).X_va(:,2), stdata(1).fitest_va, '-<', 'color',[1 0 0] );
%plot(stdata(4).X_va(:,2), stdata(4).fitest_va, '->', 'color',[0.85 0.32 0.098] );
plot(stdata(8).X_va(:,2), stdata(8).fitest_va, '-s', 'color', [1 0 1] );
%plot(stdata(12).X(:,2), stdata(12).fitest, 'c-' );
plot(stdata(16).X_va(:,2), stdata(16).fitest_va, '-+','color',[0 1 0] );

plot(stdata(1).X_bb(:,2), stdata(1).fitest_bb, '->', 'color',[0.5 0 1] );
%plot(stdata(4).X_va(:,2), stdata(4).fitest_va, '->', 'color',[0.85 0.32 0.098] );
plot(stdata(8).X_bb(:,2), stdata(8).fitest_bb, '-*', 'color', [0 1 1] );
%plot(stdata(12).X(:,2), stdata(12).fitest, 'c-' );
plot(stdata(16).X_bb(:,2), stdata(16).fitest_bb, '-o','color',[0 0 1] );


xlabel('num of frontier points to process')
ylabel('runtime (ms)')
title('Naive vs BB performance')
legend('t1(naive mp)', 't8(naive mp)', 't16(naive mp)', 't1(bb)', 't8(bb)', 't16(bb)', 'Location', 'NW')
grid on






