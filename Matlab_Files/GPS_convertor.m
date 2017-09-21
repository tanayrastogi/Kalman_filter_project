function Pos = GPS_convertor(lng,lat, ht)
% Convert GPS longitude and Latitude values into X,Y,Z codinate system
% The GPS values from the sensor are in WGS84 format

% Inputs
% lng = Longitude values in decimal degree
% lat = Latitude values in decimal degree
% ht = Altitude values 

% Convert the longitude and latitude values from deg,min,sec to degree
% decimal by decimal degree = deg + min/60 + sec/3600

% Convert the lng and lat to radians
lng = lng*(pi/180);
lat = lat*(pi/180);

% Earth radius according to WGS84
Rad = 6378137.0;

% Flatning
f = 1/298.257224;

C = 1/sqrt((cos(lat)^2)+ ((1-f)*sin(lat)^2));
S = ((1-f)^2)*C;

X = (Rad*C+ht)*cos(lat)*cos(lng);
Y = (Rad*C+ht)*cos(lat)*sin(lng);
Z = (Rad*S+ht)*sin(lat);

% Return Position
Pos = [X;Y;Z];
end