function [ output_args ] = plotMesh( X, Y, Z, R, C )
%9x9

pZ = 0;
pZ2 = 0;
for r = 0:(R-1)
    for c = 0:(C-1)
        
        if c < C - 1
            i = (r*C) + c;
            i2 = (r*C) + (c+1);
            i2 = i2+1;
            i=i+1;
           if (r==0 && c==0)
                pZ = Z(i);
                pZ2 = Z(i2);
            end
            if Z(i) < 800|| Z(i) > 1800
                Z(i) = pZ;
            end
            if Z(i2) < 800|| Z(i2) > 1800
                Z(i2) = pZ2;
            end
            
            plot3([X(i); X(i2)], [Y(i); Y(i2)],[Z(i), Z(i2)], 'b-');
        end
        if r < R - 1
            i = (r*C) + c;
            i=i+1;
            i2 = ((r+1)*C) + c;
            i2 = i2+1;
             if (r==0 && c==0)
                pZ = Z(i);
                pZ2 = Z(i2);
            end
            if Z(i) < 800|| Z(i) > 1800
                Z(i) = pZ;
            end
            if Z(i2) < 800|| Z(i2) > 1800
                Z(i2) = pZ2;
            end
             plot3([X(i); X(i2)], [Y(i); Y(i2)],[Z(i), Z(i2)], 'b-');
             
        end
        if(r == 0)
         hold on
        end
    end
    
end
hold off

%plot line = plot([X(i); X(i+1)], [Y(i); Y(i+1)], '-');