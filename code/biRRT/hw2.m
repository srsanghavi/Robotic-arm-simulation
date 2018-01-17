% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 3 that denotes question
%                       number to run.
function hw2(questionNum)

%     close all;
    
    if nargin < 1
        error('Error: please enter a question number as a parameter');
%         questionNum = 2;
    end
    
    % set up robot and initial joint configuration
	rob = createRobot();
	qStart = [0 -0.78 0 -0.78];
	xGoal = [0.5;0.0;-0.5];
      sphereCenter = [0.5;0.0;0];
    sphereRadius = 0.2;
    
    % plot robot and sphere
    rob.plot(qStart);
    hold on;	
    drawSphere(sphereCenter,sphereRadius);

    if questionNum == 1
        collision = Q1(rob,qStart,[-0.9391, -2.5286, 1.4566, -0.3478],sphereCenter,sphereRadius);
        display(['this should be 1: ',int2str(collision)])
        collision = Q1(rob,qStart,[0 -0.78 0 -1.5],sphereCenter,sphereRadius);
        display(['this should be 0: ',int2str(collision)])        
        collision = Q1(rob,qStart,[0 -0.78 0 -0.1],sphereCenter,sphereRadius);
        display(['this should be 1: ',int2str(collision)])        
    elseif questionNum == 2
        errorx = zeros(1000,100);
        i = 1;
        while(i<=100)
            
            [qMilestones,errors] = Q2(rob,sphereCenter,sphereRadius,qStart,xGoal);
            [row col] = size(errors);
            
            errorx(:,i) = transpose([errors zeros(1,1000-col)]);
%             size(errorx)
            i = i+1
        end
        errorHello = errorx;
%        errorx = hw21(2);
%     plot([1:500],mean(errorx(1:500,:)'))
        % interpolate and plot direct traj from start to goal
        qTraj = interpMilestones(qMilestones);
        rob.plot(qTraj);
        
    elseif questionNum == 3
        qMilestones = Q2(rob,sphereCenter,sphereRadius,qStart,xGoal);
        qMilestones = Q3(rob,qMilestones,sphereCenter,sphereRadius);

        % interpolate and plot direct traj from start to goal
        qTraj = interpMilestones(qMilestones);
        rob.plot(qTraj);
        
    else
        error('Error: question number out of range.');        
    end
    

end

function traj = interpMilestones(qMilestones)

    d = 0.05;
%     traj = qMilestones(1,:);
    traj = [];
    for i=2:size(qMilestones,1)
        
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
        
    end
end

function qPath = getPath(tree)

    m = 10;
    idx = size(tree,1);
    path = tree(end,1:end-1);
    
    while(idx ~= 1)
        
        curr = tree(idx,1:end-1);
        idx = tree(idx,end);
        next = tree(idx,1:end-1);
        path = [path;[linspace(curr(1),next(1),m)' linspace(curr(2),next(2),m)' linspace(curr(3),next(3),m)' linspace(curr(4),next(4),m)']];
        
    end
    qPath = path(end:-1:1,:);
    
end


function rob = createRobot()

    L(1) = Link([0 0 0 1.571]);
    L(2) = Link([0 0 0 -1.571]);
    L(3) = Link([0 0.4318 0 -1.571]);
    L(4) = Link([0 0 0.4318 1.571]);
%     L(5) = Link([0 0.4318 0 1.571]);
    
    rob = SerialLink(L, 'name', 'robot');

end

function drawSphere(position,diameter)

%     diameter = 0.1;
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
    %~ shading flat

end

