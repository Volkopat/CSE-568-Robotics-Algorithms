images = dir('*.jpg');      
n = length(images);   
for i=1:n
    currentfilename = images(i).name;
    I = imread(currentfilename);
    [row,col] = size(I)
    divider = round(row/3)
    red = I(1:divider,1:col)
    green = I(divider+1:divider*2,1:col)
    blue = I(divider*2+1:divider*3,1:col)

    finalI = I(1:divider,1:col)
    finalI(:,:,1) = blue;
    finalI(:,:,2) = green;
    finalI(:,:,3) = red;

     imwrite(finalI, strcat('image',int2str(i),'-color.jpg'));
     ssdI = im_align1(red,green,blue);
     imwrite(ssdI, strcat('image',int2str(i),'-ssd.jpg'));
     nccI = im_align2(red,green,blue);
     imwrite(nccI, strcat('image',int2str(i),'-ncc.jpg'));
     cornerI = im_align3(red,green,blue);
     imwrite(cornerI, strcat('image',int2str(i),'-corner.jpg'));
end


