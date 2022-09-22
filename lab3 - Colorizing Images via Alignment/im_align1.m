function ssdI = im_align1(red,green,blue)
    
    %Red and Green
    [rx,ry] = size(red)
    [gx,gy] = size(green)
    offsetrx = ceil((rx-50)/2)
    offsetry = ceil((ry-50)/2)
    offsetgx = ceil((gx-50)/2)
    offsetgy = ceil((gy-50)/2)
    nred = red(offsetrx:offsetrx + 50, offsetry:offsetry + 50);
    ngreen = green(offsetgx:offsetgx + 50,offsetgy:offsetgy + 50);
    
    mini = 999999
    index = 0
    dim = 1

    for i = -10:10
        for j = -10:10
            p = double(ngreen)-double(circshift(nred,[i,j]));
            sqdiff = sum(p(:).^2)
            if sqdiff < mini
                mini = sqdiff;
                index = i;
                dim = j;
            end
        end
    end
    red2 = circshift(red,[index,dim]);
    
     %Blue and Green
    [bx,by] = size(blue)
    [gx,gy] = size(green)
    
    offsetbx = ceil((bx-50)/2)
    offsetby = ceil((by-50)/2)
    offsetgx = ceil((gx-50)/2)
    offsetgy = ceil((gy-50)/2)
    nblue = blue(offsetbx:offsetbx + 50, offsetby:offsetby + 50);
    ngreen = green(offsetgx:offsetgx + 50,offsetgy:offsetgy + 50);
    
    mini = 999999
    index = 0
    dim = 1

    for i = -10:10
        for j = -10:10
            p = double(ngreen)-double(circshift(nblue,[i,j]));
            sqdiff = sum(p(:).^2)
            if sqdiff < mini
                mini = sqdiff;
                index = i;
                dim = j;
            end
        end
    end   
    blue2 = circshift(blue,[index,dim]); 
    ssdI = cat(3,blue2,green,red2);
end


