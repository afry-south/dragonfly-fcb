xyzSamples = fscanfMat( "MagnetometerSamples.txt", "%lg");

xlabel("my X");
ylabel("my Y");
zlabel("my Z"); 
param3d(xyzSamples(:,1), xyzSamples(:,2), xyzSamples(:,3));
