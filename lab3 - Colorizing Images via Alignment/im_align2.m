function nccI = im_align2(red,green,blue)
       
        %Red and Green
        [rx,ry] = size(red)
        [gx,gy] = size(green)
        offsetrx = ceil((rx-50)/2)
        offsetry = ceil((ry-50)/2)
        offsetgx = ceil((gx-50)/2)
        offsetgy = ceil((gy-50)/2)
        nred = double(red(offsetrx:offsetrx + 50, offsetry:offsetry + 50));
        ngreen = double(green(offsetgx:offsetgx + 50,offsetgy:offsetgy + 50));

        mini = 999999
        index = 0
        dim = 1

        for i = -10:10
            for j = -10:10
                mred = mean(circshift(nred,[i,j]))
                mgreen = mean(ngreen)
                nred = nred - mred
                ngreen = ngreen - mgreen
                N = dot(nred,ngreen)
                D = dot(nred.^2,ngreen.^2)
                normal = N/D
                if normal > mini
                    mini = normal;
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
        nblue = double(blue(offsetbx:offsetbx + 50, offsetby:offsetby + 50));
        ngreen = double(green(offsetgx:offsetgx + 50,offsetgy:offsetgy + 50));

        mini = 999999
        index = 0
        dim = 1

        for i = -10:10
            for j = -10:10
                mblue = mean(double(circshift(nblue,[i,j])))
                mgreen = mean(double(ngreen))
                nblue = nblue - mblue
                ngreen = ngreen - mgreen
                N = dot(nblue,ngreen)
                D = dot(nblue.^2,ngreen.^2)
                normal = N/D
                if normal > mini
                    mini = normal;
                    index = i;
                    dim = j;
                end
            end
        end   
        blue2 = circshift(blue,[index,dim]);
        nccI = cat(3 ,blue2 ,green, red);     
end