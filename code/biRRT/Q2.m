% Calculate a path from qStart to xGoal
% input: qStart -> 1x4 joint vector describing starting configuration of
%                   arm
%        xGoal -> 3x1 position describing desired position of end effector
%        sphereCenter -> 3x1 position of center of spherical obstacle
%        sphereRadius -> radius of obstacle
% output -> qMilestones -> 4xn vector of milestones. A straight-line interpolated
%                    path through these milestones should result in a
%                    collision-free path. You may output any number of
%                    milestones. The first milestone should be qStart. The
%                    last milestone should place the end effector at xGoal.


function [qMilestones, errorArr] = Q2(rob,sphereCenter,sphereRadius,qStart,xGoal)
    %STEP_SIZE = 1;
    ERROR_MAX = 0.01;
    Qs = [qStart 0];         % Qs is a tree. To generate a tree, store index of parent with the q
    nearestIndex1 = 0;
           
    
    % find the robot joint configuration for xGoal
    qGoal = rob.ikine(transl(xGoal),zeros(1,4),[1,1,1,0,0,0]);
    QsG = [qGoal 0]; 
    % plot Goal point on the graph
    posNew = xGoal; 
    hold on;
    plot3(posNew(1),posNew(2),posNew(3),'*','MarkerSize',12);
    hold off;
    drawnow;
            
    % find meansquare error between current position and goal position
    dx =   qStart - qGoal ;
    error = sqrt(sum(dx.^2));
    
    itr = 0;
%     figure;
    errorArr = [];

    % loop to generate a tree using RRT algorithm to find goal configuration     
    while (itr<1000)
        itr = itr +1 ;
        errorArr = [errorArr error];
        %generate a qRand 
        % qRand is a 1*4 matrix with each element ranging from -pi to pi 
        qRand = [randpi() randpi() randpi() randpi()];
        
        % find the closest configuration in the tree.
        [mindist,NearestIndex] = nearestQ(qRand,Qs(:,1:4));
        qNearest = Qs(NearestIndex,1:4);
        
        % check whether straight line path without collision exists between
        % the nearest configuration in tree and the random configuration
        
        % if such the path exists then add the random configuration in the
        % tree. Assign the nearest node as a parent node of the newly
        % inserted node
        
        qNew = qRand;
        %qNew = qNearest +(qRand-qNearest)./mindist.*STEPSIZE;

        if(Q1(rob,qNearest,qNew,sphereCenter,sphereRadius)==0)
            Qs = [Qs;qNew NearestIndex];
            qsIndex = NearestIndex;
            % plotting the point on graph
            % This part is not required for solving the problem
            % This is just to visualize the algorithm
%             posNew = transpose(rob.fkine(qNew) * [0;0;0;1]);
%             hold on;
%             plot3(posNew(1),posNew(2),posNew(3),'b.');
%             hold off;
%             drawnow;
            
            
        [mindist,NearestIndex] = nearestQ(qNew,QsG(:,1:4));
        qNearest = QsG(NearestIndex,1:4);
        dx =   qNearest - qNew ;
        error = sqrt(sum(dx.^2));
%         error
        if(error< ERROR_MAX) 
              break;
        end
        end
         %generate a qRand 
        % qRand is a 1*4 matrix with each element ranging from -pi to pi 
        qRand = [randpi() randpi() randpi() randpi()];
        
        % find the closest configuration in the tree.
        [mindist,NearestIndex] = nearestQ(qRand,QsG(:,1:4));
        qNearest = QsG(NearestIndex,1:4);
        nearestIndex1 = NearestIndex;
        % check whether straight line path without collision exists between
        % the nearest configuration in tree and the random configuration
        
        % if such the path exists then add the random configuration in the
        % tree. Assign the nearest node as a parent node of the newly
        % inserted node
        
        qNew1 = qRand;
        %qNew = qNearest +(qRand-qNearest)./mindist.*STEPSIZE;
        if(Q1(rob,qNearest,qNew1,sphereCenter,sphereRadius)==0)
            QsG = [QsG;qNew1 NearestIndex];
            qsGIndex = NearestIndex;
            % plotting the point on graph
            % This part is not required for solving the problem
            % This is just to visualize the algorithm
            posNew = transpose(rob.fkine(qNew1) * [0;0;0;1]);
%             hold on;
%             plot3(posNew(1),posNew(2),posNew(3),'r.');
%             hold off;
%             drawnow;
                % calculate the new meansquare error

                      % find the closest configuration in the tree.
        [mindist,NearestIndex] = nearestQ(qNew1,Qs(:,1:4));
        qNearest = Qs(NearestIndex,1:4);
        dx =   qNearest - qNew1 ;
        error = sqrt(sum(dx.^2));
        if(error< ERROR_MAX) 
              break;
        end
        end

        
        
    end
    itr
    % Traverse tree to find the path
    I = qsGIndex;
    qMilestones1 = [];
     while(I)
       q = QsG(I,1:5);
       I = q(5);
       qMilestones1 = [qMilestones1;q(1:4)];
       % plot qMilestones on the graph
%        hold on;
%        pos = (rob.fkine(q(1:4)) * [0;0;0;1]);
%        plot3(pos(1),pos(2),pos(3),'G.','MarkerSize',30);
%        hold off;
%        drawnow;
     end
    
    I= qsIndex;
    qMilestones =[];
    while(I)
       q = Qs(I,1:5);
       I = q(5);
       qMilestones = [qMilestones;q(1:4)];
       % plot qMilestones on the graph
%        hold on;
%        pos = (rob.fkine(q(1:4)) * [0;0;0;1]);
%        plot3(pos(1),pos(2),pos(3),'r.','MarkerSize',30);
%        hold off;
%        drawnow;
    end
    
    % The treen needs to be flipped vertically as we traversed it from
    % bottom to top in previous step
    qMilestones=flipud(qMilestones);
    qMilestones = [qMilestones;qMilestones1];
    % Find a path from the last point to xGoal
    % this step is required due to error not being 0
    % The final configuration in might be the configuration very
    % nearer to the Goal cofiguration but not the goal config it self
%      if(Q1(rob,qGoal,qMilestones(end,1:4),sphereCenter,sphereRadius)==0)
%              qMilestones = [qMilestones;qGoal];
%      end
%      
     errorArr = [errorArr  error];
%      plot(errorArr);
%       figure;
end


% randpi returns a random value between -pi to pi
function piv = randpi()
    piv = rand(1)*randi([-1,1])*pi;
end


% nearestQ returns an index of configuration in Tree qList which is nearest
% to the configuration q.
%  input: q    : configuration needs to be checked
%         qList: a List of qs
%  output: mindist: distance between q and nearest configuration in qList
%          index  : index of the nearest configuration in qList
function [minDist,index] = nearestQ(q,qList)
            dx1 = qList(:,1:4) - q;
            distance1 = sqrt(sum(dx1'.^2)); 
            [minDist,index] = min(distance1);
end

    