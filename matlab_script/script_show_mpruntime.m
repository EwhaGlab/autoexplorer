! open mp parallelization run time

mptime(1,:) = [25 68.9289] % 1
mptime(2,:) = [24 35.8145] % 2
mptime(3,:) = [22 31.4618] % 4
mptime(4,:) = [20 17.7212] % 8
mptime(5,:) = [21 21.0025] % 12

mptime(:,2) ./ mptime(:,1) ;