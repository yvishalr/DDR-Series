
clc;
close all;

% Physical parameters of the differential drive bot
axleLength = 0.3;
radius = 0.025;

% Initial wheel velocities and pose
rightVelocity = 3;
leftVelocity = 5;

initialTheta = 90*pi/180;
initialX = 0;
initialY = 0;

% Plotting variables
time_run = 2.5;
del_t = 0.0185;


axis([-3 3 -3 3]);
axis equal;
grid ON;
hold ON;
pause(1);
plot(initialX, initialY, 'g-o');

for i = 0:del_t:time_run
    
  omega = (rightVelocity-leftVelocity)/axleLength;
  
  if(rightVelocity ~= leftVelocity)
      
    RICC = ((rightVelocity+leftVelocity)/(rightVelocity-leftVelocity))*axleLength*0.5;
  
    [x1,y1,phi1] = differentialDriveTransformation(initialX,initialY,initialTheta,RICC,omega,del_t);
    
    flag = 1;
  else
      p = [initialX initialY];
      m = tan(wrapToPi(initialTheta));
      c = p(2)-m*p(1);
      phi1 = wrapToPi(initialTheta);
      
      if(flag == 1)
      iterate = norm([initialX x1]-[initialY y1])*abs(cos(phi1))*radius*rightVelocity/4;
      end
      
      flag = 0;

      if -pi/2 < initialTheta && initialTheta < pi/2
      
      x1 = p(1)+iterate;
      y1 = m*x1+c;
      
      elseif initialTheta == pi/2 
              
      x1 = p(1);
      y1 = p(2)+ iterate;
      
      elseif initialTheta == -pi/2
              
      x1 = p(1);
      y1 = p(2)-iterate;
      
      else
          
      x1 = p(1)-iterate;
      y1 = m*x1+c;
          
      end
  end
  
  plot(x1,y1,'g-o');
  pause(0.03);
  
  initialX = x1;
  initialY = y1;
  initialTheta = phi1;
  
  % Add required path changes at desired timestep here
  if(i == del_t*25)
      temp = rightVelocity;
      rightVelocity = leftVelocity;
      leftVelocity = temp;
  elseif(i == del_t*45)
      rightVelocity = leftVelocity;
  elseif(i == del_t*80)
      leftVelocity = leftVelocity + 2.5;
      rightVelocity = rightVelocity + 1.5;
  end
end

% Calculation of forward kinematics
function [x,y,phi]=differentialDriveTransformation(a,b,theta,R,w,t)

ICC = [a-R*sin(theta);b+R*cos(theta)];
plot(ICC(1),ICC(2),'r-*');
text(ICC(1),ICC(2),'ICC','Color',[1 0 0],'VerticalAlignment','Top',...
    'HorizontalAlignment','Center');

Rotate = [cos(w*t) -sin(w*t) 0;sin(w*t) cos(w*t) 0;0 0 1];
poseICC = [a-ICC(1);b-ICC(2);theta];

FinalPose = Rotate*poseICC + [ICC(1);ICC(2);w*t];

x = FinalPose(1);
y = FinalPose(2);
phi = FinalPose(3);

end



















