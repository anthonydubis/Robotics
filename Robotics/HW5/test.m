function test( R )
% Test whatever you want here
    while true
        img_path = 'http://192.168.0.102/snapshot.cgi?user=admin&pwd=&resolution=10&rate=0';
        img = imread(img_path); 
        imshow(img);
    end
end

