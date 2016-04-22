D = readtable('data.csv');




M =[D.frame, D.x, D.y, D.z];
for frame = 1:850
    index = M(:,1) == frame;
    P = M(index, 2:4);
    x = P(:,1);
    y = P(:,2);
    z = P(:,3);
    dx = 10;
    dy = 10;
    x_bound = floor(min(x)):dx:ceil(max(x));
    y_bound = floor(min(y)):dy:ceil(max(y));
    [X,Y] = meshgrid(x_bound, y_bound);
    Z = griddata(x,y,z, X, Y);
  
   % surf(X, Y, Z);
   % axis([0 500 0 500 900 1400]);
 %   view([180, 60]);

    
  title('Cloth Depth Data (mm)');
xlabel('Pixel Col');
ylabel('Pixel Row');
zlabel('Depth (mm)');
%figure 

 %plot3(x, y, z, 'bo');

 plotMesh(x,y,z,9,9);

   title('Cloth Depth Data (mm)');
xlabel('Pixel Col');
ylabel('Pixel Row');
zlabel('Depth (mm)');
 axis([0 500 0 500 900 1400]);
 view([-135, 45]);
 pause(.001);
   % pause(0.01);
   % pause(10);
   
end

