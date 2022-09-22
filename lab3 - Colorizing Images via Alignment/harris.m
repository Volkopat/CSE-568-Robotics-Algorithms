function harrisI = harris(red,green,blue)

    dx = [-1 0 1;-1 0 1;-1 0 1]
    dy = [-1 -1 -1;0 0 0;1 1 1]

    Ix = conv2(double(blue),dx,'same')
    Iy = conv2(double(blue),dy,'same')

    sigma = 1
    radius = 1
    order = (2*radius+1)^2
    threshold = 200
    sum = 0

    dim = max(1,fix(6*sigma))
    m = dim
    n = dim
    [h1,h2] = meshgrid(-(m-1)/2:(m-1)/2,-(n-1)/2:(n-1)/2)
    hg = exp(-(h1.^2+h2.^2)/(2*sigma^2))

    [a,b] = size(hg)

    for i=1:a
        for j=1:b
            sum = sum+hg(i,j)
        end
    end

    gauss = hg ./sum

    Ix2 = conv2(double(Ix.^2),gauss,'same')
    Iy2 = conv2(double(Iy.^2),gauss,'same')
    Ixy = conv2(double(Ix.*Iy),gauss,'same')

    R = (Ix2.*Iy2 - Ixy.^2) ./ (Ix2+Iy2 +eps)

    mx = ordfilt2(R, order^2, ones(order))
    harris_points = (R==mx) & (R > threshold)

    [row,col] = find(harris_points)
    subplot(1, 3, 1);
    imshow(blue), hold on,
    plot(col,row,'ys')

    Ix = conv2(double(green),dx,'same')
    Iy = conv2(double(green),dy,'same')

    sigma = 1
    radius = 1
    order = (2*radius+1)^2
    threshold = 200
    sum = 0

    dim = max(1,fix(6*sigma))
    m = dim
    n = dim
    [h1,h2] = meshgrid(-(m-1)/2:(m-1)/2,-(n-1)/2:(n-1)/2)
    hg = exp(-(h1.^2+h2.^2)/(2*sigma^2))

    [a,b] = size(hg)

    for i=1:a
        for j=1:b
            sum = sum+hg(i,j)
        end
    end

    gauss = hg ./sum

    Ix2 = conv2(double(Ix.^2),gauss,'same')
    Iy2 = conv2(double(Iy.^2),gauss,'same')
    Ixy = conv2(double(Ix.*Iy),gauss,'same')

    R = (Ix2.*Iy2 - Ixy.^2) ./ (Ix2+Iy2 +eps)

    mx = ordfilt2(R, order^2, ones(order))
    harris_points = (R==mx) & (R > threshold)

    [row,col] = find(harris_points)
    subplot(1, 3, 2);
    imshow(green), hold on,
    plot(col,row,'ys')

    Ix = conv2(double(red),dx,'same')
    Iy = conv2(double(red),dy,'same')

    sigma = 1
    radius = 1
    order = (2*radius+1)^2
    threshold = 10000
    sum = 0

    dim = max(1,fix(6*sigma))
    m = dim
    n = dim
    [h1,h2] = meshgrid(-(m-1)/2:(m-1)/2,-(n-1)/2:(n-1)/2)
    hg = exp(-(h1.^2+h2.^2)/(2*sigma^2))

    [a,b] = size(hg)

    for i=1:a
        for j=1:b
            sum = sum+hg(i,j)
        end
    end

    gauss = hg ./sum

    Ix2 = conv2(double(Ix.^2),gauss,'same')
    Iy2 = conv2(double(Iy.^2),gauss,'same')
    Ixy = conv2(double(Ix.*Iy),gauss,'same')

    R = (Ix2.*Iy2 - Ixy.^2) ./ (Ix2+Iy2 +eps)

    mx = ordfilt2(R, order^2, ones(order))
    harris_points = (R==mx) & (R > threshold)

    [row,col] = find(harris_points)
    subplot(1, 3, 3);
    imshow(red), hold on,
    plot(col,row,'ys')

    harrisI = cat(3,blue,green,red)
    plot(col,row,'ys')
end


