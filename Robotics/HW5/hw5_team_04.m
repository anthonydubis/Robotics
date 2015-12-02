function [ output_args ] = hw5_team_04( input_args )

image = imread('http://[IP_ADDRESS]/img/snapshot.cgi?');
imshow(image);

image = imread('http://[IP_ADDRESS]/snapshot.cgi?');
imshow(image);

imread('http://[IP_ADDRESS]/snapshot.cgi?user=admin&pwd=&resolution=16&rate=0');
imshow(image);

end

