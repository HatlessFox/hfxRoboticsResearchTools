function displayPoints(Map,c,ppolar) 



if(nargin==3 && ppolar==1)
    polar(Map(:,1), Map(:,2), ['.' c]); 
else

    if(size(Map,1) == 3)
        plot3(Map(:,1),Map(:,2),Map(:,3), ['.' c], 'MarkerSize', 6);
    else
        plot(Map(:,1),Map(:,2), ['.' c], 'MarkerSize', 6);
    end

end