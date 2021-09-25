function costmapCallback( src, message )

    global I;
    global point ;
    global ox_ ;
    global oy_ ;
    global res ;
    global w;
    global h;
    global orig ;

    w = message.Info.Width ;
    h = message.Info.Height ;
    I = reshape(message.Data, w, h)' ;

    ox_ = message.Info.Origin.Position.X ;
    oy_ = message.Info.Origin.Position.Y ;
    res = double(message.Info.Resolution) ;

    %oidx = floor( (0-oy_)/res ) * w + floor( (0-ox_)/res ) ;
    %[oy, ox] = ind2sub(size(I), oidx) ;
    
    oy = ( (0-oy_)/res ) ; 
    ox = ( (0-ox_)/res ) ; 
    orig =  [ox oy];
    
end