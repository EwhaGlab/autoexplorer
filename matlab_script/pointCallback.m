function pointCallback(src, message)

    global point ;
    global ox_ ;
    global oy_ ;
    global res ;
    global w ;
    global h ;
    global I ;
    global orig ;
    global pts ;
    global pts_ ;
    
    point = [ message.Point.X message.Point.Y ] ;
    %[fy, fx] = ind2sub(size(I), idx) ;
    
    fy = message.Point.Y ; %( (message.Point.Y - oy_) /res )  ;
    fx = message.Point.X ; %( (message.Point.X - ox_) /res ) ;
    pts_ = [pts_; [fx fy] ];
    
%     
%     for idx=1:size(1,pt_)
%         px = (pts_(idx,1) - ox_)/res ;
%         py = (pts_(idx,2) - oy_)/res ;
% 
% 		for( ridx =-10:10 )
%             for( cidx=-10:10 )
% 				idx = px_c + cidx + (py_c + ridx) * width ;
% 				cost = Data[idx] ;
%             end
%         end
% 			
% 
%         roi = I( px-[-10:10], py-[-10:10]) ;
% 
%         figure(2 + idx)
%         imshow(roi) ;
%     end
end