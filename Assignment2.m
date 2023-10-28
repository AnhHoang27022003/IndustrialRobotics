 clf
 clc
 clear all
 hold on
 axis ([-2 2 -2 2 0 2.8])

%call robots.
    r1 = UR3;
    r1.model.base = eye(4)*transl(-1.5,-0.2,2.075)*trotx(pi);
    r1.model.animate([pi/2,-pi/2,0,-pi/2,0,0])
    r2 = LinearCRB1100; 

%kitchen environment
 kitchen_environment = PlaceObject('Kitchen environment.ply',[0,0,0]);

%pan
    [panf,panv,pandata] = plyread('pan.ply','tri');
    panVertexCount = size(panv,1);
    panMidpoint = sum(panv)/panVertexCount;
    panVerts = panv - repmat(panMidpoint, panVertexCount, 1); 
    panVertexColors = [ones(height(panv),1)*65/255, ones(height(panv),1)*65/255,  ones(height(panv),1)*65/255];
    panMesh_h = trisurf(panf, panVerts(:,1), panVerts(:,2), panVerts(:,3) ...
                        ,'FaceVertexCData', panVertexColors, 'EdgeColor','interp','EdgeLighting','flat');
    panplotpose = transl(-1.5,0.05,1.3);   
    panupdatedPoints = [panplotpose * [panVerts,ones(panVertexCount,1)]']';
    panMesh_h.Vertices = panupdatedPoints(:,1:3);
    drawnow();
%pan2
    [pan2f,pan2v,pan2data] = plyread('pan2.ply','tri');
    pan2VertexCount = size(pan2v,1);
    pan2Midpoint = sum(pan2v)/pan2VertexCount;
    pan2Verts = pan2v - repmat(pan2Midpoint, pan2VertexCount, 1);  
    pan2VertexColors = [ones(height(pan2v),1)*50/255, ones(height(pan2v),1)*50/255,  ones(height(pan2v),1)*50/255];
    pan2Mesh_h = trisurf(pan2f, pan2Verts(:,1), pan2Verts(:,2), pan2Verts(:,3) ...
                        ,'FaceVertexCData', pan2VertexColors, 'EdgeColor','interp','EdgeLighting','flat');
    pan2plotpose = transl(-1.4,-0.5,1.325)*trotz(pi/2);
    pan2updatedPoints = [pan2plotpose * [pan2Verts,ones(pan2VertexCount,1)]']';
    pan2Mesh_h.Vertices = pan2updatedPoints(:,1:3);
    drawnow();
%sauce
    [saucef,saucev,saucedata] = plyread('sauce.ply','tri');
    sauceVertexCount = size(saucev,1);
    sauceMidpoint = sum(saucev)/sauceVertexCount;
    sauceVerts = saucev - repmat(sauceMidpoint, sauceVertexCount, 1); 
    sauceVertexColors = [ones(height(saucev),1), ones(height(saucev),1),  zeros(height(saucev),1)];
    sauceMesh_h = trisurf(saucef, sauceVerts(:,1), sauceVerts(:,2), sauceVerts(:,3) ...
                        ,'FaceVertexCData', sauceVertexColors, 'EdgeColor','interp','EdgeLighting','flat');
    sauceplotpose = transl(-1.2,-0.8,1.3);
    sauceupdatedPoints = [sauceplotpose * [sauceVerts,ones(sauceVertexCount,1)]']';
    sauceMesh_h.Vertices = sauceupdatedPoints(:,1:3);
    drawnow();
%man   
    global manVertexCount
    global manMidpoint
    global manVerts
    global manVertexColors
    global manMesh_h
    global manplotpose
    global manupdatedPoints
    global manf
    global manv
    global mandata
    [manf,manv,mandata] = plyread('Standing Man.ply','tri');
    manVertexCount = size(manv,1);
    manMidpoint = sum(manv)/manVertexCount;
    manVerts = manv - repmat(manMidpoint, manVertexCount, 1); 
    manVertexColors = [zeros(height(manv),1), zeros(height(manv),1),  ones(height(manv),1)];
    manMesh_h = trisurf(manf, manVerts(:,1), manVerts(:,2), manVerts(:,3) ...
                        ,'FaceVertexCData', manVertexColors, 'EdgeColor','interp','EdgeLighting','flat');
    manplotpose = eye(4)*transl(-0.2,0,1000)*trotz(-pi/2);
    manupdatedPoints = [manplotpose * [manVerts,ones(manVertexCount,1)]']';
    manMesh_h.Vertices = manupdatedPoints(:,1:3);
    drawnow();
%MAN'S CUBEPOINTS
    global X
    global Y
    global sizeMat
    global Z
    global ManCubePoints   
    [X,Y] = meshgrid(-1.2:0.1:-0.8,-0.4:0.1:0.4);
    sizeMat = size(X);
    Z = repmat(1000,sizeMat(1),sizeMat(2));
    ManCubePoints = [X(:),Y(:),Z(:)];
    ManCubePoints = ManCubePoints + repmat([0,0,0],size(ManCubePoints,1),1);
    %ManCubePoints_h = plot3(ManCubePoints(:,1),ManCubePoints(:,2),ManCubePoints(:,3),'r.');

%MAN'S ELLIPSOID
    global ManCenterPoint
    global ManRadii
    global x
    global y
    global z
    ManCenterPoint = [0,-1.9,1000];
    ManRadii = [0.3,0.2,1.3];
    [x,y,z] = ellipsoid(ManCenterPoint(1), ManCenterPoint(2), ManCenterPoint(3), ManRadii(1), ManRadii(2), ManRadii(3));
    ManEllipsoid_h = plot3(x,y,z);

%Light Curtain
    [X1,Z1] = meshgrid(-2:0.2:2,0:0.2:2.8);
    sizeMat1 = size(X1);
    Y1 = repmat(-2,sizeMat1(1),sizeMat1(2));
    CurtainCubePoints = [X1(:),Y1(:),Z1(:)];
    CurtainCubePoints = CurtainCubePoints + repmat([0,0,0],size(CurtainCubePoints,1),1);
    CurtainCubePoints_h = plot3(CurtainCubePoints(:,1),CurtainCubePoints(:,2),CurtainCubePoints(:,3),'b.');

% Link 1
centerPoint1 = [0,0,0.05];
radii1 = [0.1,0.1,0.125];
[X,Y,Z] = ellipsoid( centerPoint1(1), centerPoint1(2), centerPoint1(3), radii1(1), radii1(2), radii1(3) );
r1.model.points{1} = [X(:),Y(:),Z(:)];
warning off
r1.model.faces{1} = delaunay(r1.model.points{1}); 

% Link 2
centerPoint2 = [-0.05,0,0];
radii2 = [0.125,0.1,0.1];
[X,Y,Z] = ellipsoid( centerPoint2(1), centerPoint2(2), centerPoint2(3), radii2(1), radii2(2), radii2(3) );
r1.model.points{2} = [X(:),Y(:),Z(:)];
warning off
r1.model.faces{2} = delaunay(r1.model.points{2}); 

% Link 3
centerPoint3 = [0,0,0];
radii3 = [0.15,0.1,0.1];
[X,Y,Z] = ellipsoid( centerPoint3(1), centerPoint3(2), centerPoint3(3), radii3(1), radii3(2), radii3(3) );
r1.model.points{3} = [X(:),Y(:),Z(:)];
warning off
r1.model.faces{3} = delaunay(r1.model.points{3}); 

% Link 4
centerPoint4 = [0,0,0];
radii4 = [0.1,0.125,0.1];
[X,Y,Z] = ellipsoid( centerPoint4(1), centerPoint4(2), centerPoint4(3), radii4(1), radii4(2), radii4(3) );
r1.model.points{4} = [X(:),Y(:),Z(:)];
warning off
r1.model.faces{4} = delaunay(r1.model.points{4});

% Link 5
centerPoint5 = [0,0,0];
radii5 = [0.08,0.08,0.125];
[X,Y,Z] = ellipsoid( centerPoint5(1), centerPoint5(2), centerPoint5(3), radii5(1), radii5(2), radii5(3) );
r1.model.points{5} = [X(:),Y(:),Z(:)];
warning off
r1.model.faces{5} = delaunay(r1.model.points{5}); 

% Link 6
centerPoint6 = [0,0,0];
radii6 = [0.1,0.1,0.11];
[X,Y,Z] = ellipsoid( centerPoint6(1), centerPoint6(2), centerPoint6(3), radii6(1), radii6(2), radii6(3) );
r1.model.points{6} = [X(:),Y(:),Z(:)];
warning off
r1.model.faces{6} = delaunay(r1.model.points{6});

% Link 7
centerPoint7 = [0,0,0];
radii7 = [0.02,0.02,0.02];
[X,Y,Z] = ellipsoid( centerPoint7(1), centerPoint7(2), centerPoint7(3), radii7(1), radii7(2), radii7(3) );
r1.model.points{7} = [X(:),Y(:),Z(:)];
warning off
r1.model.faces{7} = delaunay(r1.model.points{7});
warning on

tr = zeros(4,4,r1.model.n+1);
tr(:,:,1) = r1.model.base;
L = r1.model.links;
%r1.model.plot3d([pi/2,-pi/2,0,-pi/2,0,0])

%% APP DESIGN VARIABLES FOR TEACH FUNCTION
app = app1;
global spawn_counter
spawn_counter = 0;
global Q1
Q1 = 0;
global Q2
Q2 = 0;
global Q3
Q3 = 0;
global Q4
Q4 = 0;
global Q5
Q5 = 0;
global Q6
Q6 = 0;
global X_Coordinate
X_Coordinate = 0;
global Y_Coordinate
Y_Coordinate = 0;
global Z_Coordinate
Z_Coordinate = 0;
global counter
counter = 1;
global previous_counter
previous_counter = 0;
global counter2
counter2 = 1;
global previous_counter2
previous_counter2 = 0;

 %% INITIALISE FINGERS SERIAL LINKS 
      L1 = Link('d',0,'a',0.5,'alpha',0,'qlim',[0 pi/2]);   
      finger1 = SerialLink([L1],'name','myFinger1');

      L2 = Link('d',0,'a',-0.5,'alpha',0,'qlim',[-pi/2 0]);
      finger2 = SerialLink([L2],'name','myFinger2');

      L3 = Link('d',0,'a',0.5,'alpha',0,'qlim',[0 pi/2]);   
      finger3 = SerialLink([L3],'name','myFinger1');

      L4 = Link('d',0,'a',-0.5,'alpha',0,'qlim',[-pi/2 0]);
      finger4 = SerialLink([L4],'name','myFinger2');

%% FINGER PLY FILES
% finger 1
    [finger1f,finger1v,finger1data] = plyread('finger1.ply','tri');
    finger1VertexCount = size(finger1v,1);
    finger1Midpoint = sum(finger1v)/finger1VertexCount;
    finger1Verts = finger1v - repmat(finger1Midpoint, finger1VertexCount, 1);
    finger1VertexColors = [ones(height(finger1v),1), zeros(height(finger1v),1),  zeros(height(finger1v),1)];
    finger1Mesh_h = trisurf(finger1f, finger1Verts(:,1), finger1Verts(:,2), finger1Verts(:,3) ...
        ,'FaceVertexCData', finger1VertexColors, 'EdgeColor','interp','EdgeLighting','flat');
    finger1plotpose = finger1.base.T;
% finger 2
    [finger2f,finger2v,finger2data] = plyread('finger2.ply','tri');
    finger2VertexCount = size(finger2v,1);
    finger2Midpoint = sum(finger2v)/finger2VertexCount;
    finger2Verts = finger2v - repmat(finger2Midpoint, finger2VertexCount, 1);
    finger2VertexColors = [ones(height(finger2v),1), zeros(height(finger2v),1),  zeros(height(finger2v),1)];
    finger2Mesh_h = trisurf(finger2f, finger2Verts(:,1), finger2Verts(:,2), finger2Verts(:,3) ...
        ,'FaceVertexCData', finger2VertexColors, 'EdgeColor','interp','EdgeLighting','flat');
    finger2plotpose = finger1.base.T;

% finger 3
    [finger3f,finger3v,finger3data] = plyread('finger3.ply','tri');
    finger3VertexCount = size(finger3v,1);
    finger3Midpoint = sum(finger3v)/finger3VertexCount;
    finger3Verts = finger3v - repmat(finger3Midpoint, finger3VertexCount, 1);
    finger3VertexColors = [ones(height(finger3v),1), zeros(height(finger3v),1),  zeros(height(finger3v),1)];
    finger3Mesh_h = trisurf(finger3f, finger3Verts(:,1), finger3Verts(:,2), finger3Verts(:,3) ...
        ,'FaceVertexCData', finger3VertexColors, 'EdgeColor','interp','EdgeLighting','flat');
    finger3plotpose = finger3.base.T;
% finger 4
    [finger4f,finger4v,finger4data] = plyread('finger4.ply','tri');
    finger4VertexCount = size(finger4v,1);
    finger4Midpoint = sum(finger4v)/finger4VertexCount;
    finger4Verts = finger4v - repmat(finger4Midpoint, finger4VertexCount, 1);
    finger4VertexColors = [ones(height(finger4v),1), zeros(height(finger4v),1),  zeros(height(finger4v),1)];
    finger4Mesh_h = trisurf(finger4f, finger4Verts(:,1), finger4Verts(:,2), finger4Verts(:,3) ...
        ,'FaceVertexCData', finger4VertexColors, 'EdgeColor','interp','EdgeLighting','flat');
    finger4plotpose = finger4.base.T;

%% STATE MACHINE VARIABLES
    state = 0;        %INITIALISED AS 0
    stored_state = 1; %INITIALISED AS 1 BECAUSE FIRST STORED STATE IT GOES TO IS STATE 1
    stored_i = 0;     %INITIALISED AS 0 BECAUSE FIRST ITERATION IT GOES THROUGH IT 0+1 = 1
    global estop_state
    estop_state = 0;  %INITIALISED AS 0
    global resume_state
    resume_state = 0; %INITIALISED AS 0


%% THE BIG WHILE LOOP
while (1)
   drawnow()
    %% STATE 0: WAITING TO BE RESUMED
      if state == 0    
         disp('STATE 0')
         if previous_counter == counter 
             r1.model.animate([Q1,Q2,Q3,Q4,Q5,Q6]);
             q1 = r1.model.getpos();
            finger1.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
            finger2.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
            finger1plotpose = finger1.fkine(-30/30*pi/2.2).T*transl(-0.5,0,0);
            finger1plotpose = finger1plotpose*transl(-0.03,-0.005,0);
            finger1updatedPoints = [finger1plotpose * [finger1Verts,ones(finger1VertexCount,1)]']';
            finger1Mesh_h.Vertices = finger1updatedPoints(:,1:3);
            drawnow();
            finger1plotpose = finger1plotpose*transl(0.03,0.005,0);
            finger2plotpose = finger2.fkine(30/30*pi/2.2).T*transl(0.5,0,0);
            finger2plotpose = finger2plotpose*transl(0.03,-0.005,0);
            finger2updatedPoints = [finger2plotpose * [finger2Verts,ones(finger2VertexCount,1)]']';
            finger2Mesh_h.Vertices = finger2updatedPoints(:,1:3);
            drawnow();
            finger2plotpose = finger2plotpose*transl(-0.03,0.005,0);
             counter = counter+1;
         end
         if previous_counter2 == counter2 
             r1.model.animate(r1.model.ikine(transl(X_Coordinate,Y_Coordinate,Z_Coordinate),'q0',[0,0,0,0,0,0],'mask',[1,1,1,0,0,0]));
            q1 = r1.model.getpos();
            finger1.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
            finger2.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
            finger1plotpose = finger1.fkine(-30/30*pi/2.2).T*transl(-0.5,0,0);
            finger1plotpose = finger1plotpose*transl(-0.03,-0.005,0);
            finger1updatedPoints = [finger1plotpose * [finger1Verts,ones(finger1VertexCount,1)]']';
            finger1Mesh_h.Vertices = finger1updatedPoints(:,1:3);
            drawnow();
            finger1plotpose = finger1plotpose*transl(0.03,0.005,0);
            finger2plotpose = finger2.fkine(30/30*pi/2.2).T*transl(0.5,0,0);
            finger2plotpose = finger2plotpose*transl(0.03,-0.005,0);
            finger2updatedPoints = [finger2plotpose * [finger2Verts,ones(finger2VertexCount,1)]']';
            finger2Mesh_h.Vertices = finger2updatedPoints(:,1:3);
            drawnow();
            finger2plotpose = finger2plotpose*transl(-0.03,0.005,0);
             counter2 = counter2+1;
         end
         if estop_state == 0
            if resume_state == 1 
                 state = stored_state;
                 resume_state = 0;
            end
         end
      end
    
    %% STATE 1: PICK UP PAN 1 + PAN 2
      if state == 1
          disp('STATE 1')
          stored_state = 1;
          pan2plotpose = transl(-1.4,-0.5,1.425)*trotz(pi/2);
          if stored_i == 0
            qCurrent1 = [pi/2,-pi/2,0,-pi/2,0,0];
            qEnd1 = r1.model.ikine(pan2plotpose*transl(0,-0.25,0.25)*trotx(pi), 'q0', [pi/2,-pi/2,0,-pi/2,0,0]);
          else
            qCurrent1 = r1.model.getpos();
            qEnd1 = [pi/2,-pi,0,-pi/2,0,0];
            stored_i = 12;
          end
          JTrajQMatrix1 = jtraj(qCurrent1, qEnd1, 50);
          panplotpose = transl(-1.5,0.05,1.3);  
          qCurrent2 = r2.model.getpos();
          qEnd2 = r2.model.ikine(panplotpose*transl(0,0.2,0.15)*trotx(pi));
          JTrajQMatrix2 = jtraj(qCurrent2, qEnd2, 50);
          for i = (stored_i + 1):50                      
                 if state == 1 
                         stored_i = i;
                         r1.model.animate(JTrajQMatrix1(i,:));
                         for j = 1 : 6
                                tr(:,:,j+1) = tr(:,:,j) * trotz(JTrajQMatrix1(i,j)) * transl(0,0,L(j).d) * transl(L(j).a,0,0) * trotx(L(j).alpha);
                                cubePointsAndOnes = [inv(tr(:,:,j)) * [ManCubePoints,ones(size(ManCubePoints,1),1)]']';
                                updatedCubePoints = cubePointsAndOnes(:,1:3);
                            if j == 1
                                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint1, radii1);
                            elseif j == 2
                                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint2, radii2);
                            elseif j == 3
                                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint3, radii3);
                            elseif j == 4
                                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint4, radii4);
                            elseif j == 5
                                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint5, radii5);
                            elseif j == 6
                                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint6, radii6);
                            end
                                pointsInside = find(min(algebraicDist) < 1);
                                if pointsInside > 0
                                   state = 0;
                                   resume_state = 1;
                                   disp('ROBOT IS GOING TO COLLIDE WITH HUMAN. RECALCULATING ROUTE.');
                                end
                         end
                         q1 = r1.model.getpos();
                         finger1.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
                         finger2.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
                         finger1plotpose = finger1.fkine(-1/50*pi/2.2).T*transl(-0.5,0,0);
                         finger1plotpose = finger1plotpose*transl(-0.03,-0.005,0);
                         finger1updatedPoints = [finger1plotpose * [finger1Verts,ones(finger1VertexCount,1)]']';
                         finger1Mesh_h.Vertices = finger1updatedPoints(:,1:3);
                         drawnow();
                         finger1plotpose = finger1plotpose*transl(0.03,0.005,0);
                         finger2plotpose = finger2.fkine(1/50*pi/2.2).T*transl(0.5,0,0);
                         finger2plotpose = finger2plotpose*transl(0.03,-0.005,0);
                         finger2updatedPoints = [finger2plotpose * [finger2Verts,ones(finger2VertexCount,1)]']';
                         finger2Mesh_h.Vertices = finger2updatedPoints(:,1:3);
                         drawnow();
                         finger2plotpose = finger2plotpose*transl(-0.03,0.005,0);
                         r2.model.animate(JTrajQMatrix2(i,:));
                         q2 = r2.model.getpos();
                         finger3.base = r2.model.fkine(q2).T*transl(0,0,0.023)*trotx(pi/2);
                         finger4.base = r2.model.fkine(q2).T*transl(0,0,0.023)*trotx(pi/2);
                         finger3plotpose = finger3.fkine(-i/50*pi/2.2).T*transl(-0.5,0,0);
                         finger3plotpose = finger3plotpose*transl(-0.03,-0.005,0);
                         finger3updatedPoints = [finger3plotpose * [finger3Verts,ones(finger3VertexCount,1)]']';
                         finger3Mesh_h.Vertices = finger3updatedPoints(:,1:3);
                         drawnow();
                         finger3plotpose = finger3plotpose*transl(0.03,0.005,0);
                         finger4plotpose = finger4.fkine(i/50*pi/2.2).T*transl(0.5,0,0);
                         finger4plotpose = finger4plotpose*transl(0.03,-0.005,0);
                         finger4updatedPoints = [finger4plotpose * [finger4Verts,ones(finger4VertexCount,1)]']';
                         finger4Mesh_h.Vertices = finger4updatedPoints(:,1:3);
                         drawnow();
                         finger4plotpose = finger4plotpose*transl(-0.03,0.005,0);                  
                         if estop_state == 1
                             state = 0;
                         end
                         if i == 50
                             state = 2;
                             stored_i = 0;
                         end
                 end
          end
      end
    
    %% STATE 2: PICK UP PAN 1
    if state == 2
         disp('STATE 2')
         stored_state = 2;
         pan2plotpose = transl(-1.4,-0.5,1.325)*trotz(pi/2);
         qCurrent1 = r1.model.getpos();
         qEnd1 = r1.model.ikine(pan2plotpose*transl(0,-0.25,0.25)*trotx(pi), 'q0',[pi/2,-pi/2,0,-pi/2,0,0] );
         JTrajQMatrix1 = jtraj(qCurrent1, qEnd1, 50);
         for i = (stored_i + 1):50
             if state == 2
                 r1.model.animate(JTrajQMatrix1(i,:));
                 q1 = r1.model.getpos();
                 finger1.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
                 finger2.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
                 finger1plotpose = finger1.fkine(-i/50*pi/2.2).T*transl(-0.5,0,0);
                 finger1plotpose = finger1plotpose*transl(-0.03,-0.005,0);
                 finger1updatedPoints = [finger1plotpose * [finger1Verts,ones(finger1VertexCount,1)]']';
                 finger1Mesh_h.Vertices = finger1updatedPoints(:,1:3);
                 drawnow();
                 finger1plotpose = finger1plotpose*transl(0.03,0.005,0);
                 finger2plotpose = finger2.fkine(i/50*pi/2.2).T*transl(0.5,0,0);
                 finger2plotpose = finger2plotpose*transl(0.03,-0.005,0);
                 finger2updatedPoints = [finger2plotpose * [finger2Verts,ones(finger2VertexCount,1)]']';
                 finger2Mesh_h.Vertices = finger2updatedPoints(:,1:3);
                 drawnow();
                 finger2plotpose = finger2plotpose*transl(-0.03,0.005,0);            
                 stored_i = i;
                 if estop_state == 1
                        state = 0;
                 end
                 if i == 50
                     state = 3;
                     stored_i = 0;
                 end
             end
         end
    end
    
    %% STATE 3:  POUR MEAT FROM PAN 2 INTO PAN 1
    if state == 3
        disp('STATE 3')
        stored_state = 3;
        pan2plotpose = transl(-1.4,-0.4,1.5)*trotz(pi/2);
        qCurrent1 = r1.model.getpos();
        qEnd1 = r1.model.ikine(pan2plotpose*transl(0,-0.25,0.25)*trotx(pi),'q0', qCurrent1);
        JTrajQMatrix1 = jtraj(qCurrent1, qEnd1, 30);
        for i = (stored_i + 1):30
              if state == 3
                     r1.model.animate(JTrajQMatrix1(i,:));
                     q1 = r1.model.getpos();
                     finger1.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
                     finger2.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
                     finger1plotpose = finger1.fkine(-30/30*pi/2.2).T*transl(-0.5,0,0);
                     finger1plotpose = finger1plotpose*transl(-0.03,-0.005,0);
                     finger1updatedPoints = [finger1plotpose * [finger1Verts,ones(finger1VertexCount,1)]']';
                     finger1Mesh_h.Vertices = finger1updatedPoints(:,1:3);
                     drawnow();
                     finger1plotpose = finger1plotpose*transl(0.03,0.005,0);
                     finger2plotpose = finger2.fkine(30/30*pi/2.2).T*transl(0.5,0,0);
                     finger2plotpose = finger2plotpose*transl(0.03,-0.005,0);
                     finger2updatedPoints = [finger2plotpose * [finger2Verts,ones(finger2VertexCount,1)]']';
                     finger2Mesh_h.Vertices = finger2updatedPoints(:,1:3);
                     drawnow();
                     finger2plotpose = finger2plotpose*transl(-0.03,0.005,0);
                     pan2plotpose = r1.model.fkine(q1).T*trotx(-pi)*transl(0,0.25,-0.25);
                     pan2updatedPoints = [pan2plotpose * [pan2Verts,ones(pan2VertexCount,1)]']';
                     pan2Mesh_h.Vertices = pan2updatedPoints(:,1:3);
                     drawnow();
                     stored_i = i;
                     if estop_state == 1
                         state = 0;
                     end
                     if i == 30
                         state = 4;
                         stored_i = 0;
                     end
              end
         end
    end
    
    %% STATE 4: POUR MEAT FROM PAN 2 INTO PAN 1
    if state == 4
       disp('STATE 4')
       stored_state = 4;
       qCurrent1 = r1.model.getpos();  
       qEnd1 = [qCurrent1(1),qCurrent1(2)+0.4,qCurrent1(3)-1.1,qCurrent1(4),qCurrent1(5),qCurrent1(6)];
       JTrajQMatrix1 = jtraj(qCurrent1, qEnd1, 30);
       for i = (stored_i + 1):30
          if state == 4
             r1.model.animate(JTrajQMatrix1(i,:));
             q1 = r1.model.getpos();
             finger1.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
             finger2.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
             finger1plotpose = finger1.fkine(-30/30*pi/2.2).T*transl(-0.5,0,0);
             finger1plotpose = finger1plotpose*transl(-0.03,-0.005,0);
             finger1updatedPoints = [finger1plotpose * [finger1Verts,ones(finger1VertexCount,1)]']';
             finger1Mesh_h.Vertices = finger1updatedPoints(:,1:3);
             drawnow();
             finger1plotpose = finger1plotpose*transl(0.03,0.005,0);
             finger2plotpose = finger2.fkine(30/30*pi/2.2).T*transl(0.5,0,0);
             finger2plotpose = finger2plotpose*transl(0.03,-0.005,0);
             finger2updatedPoints = [finger2plotpose * [finger2Verts,ones(finger2VertexCount,1)]']';
             finger2Mesh_h.Vertices = finger2updatedPoints(:,1:3);
             drawnow();
             finger2plotpose = finger2plotpose*transl(-0.03,0.005,0);
             pan2plotpose = r1.model.fkine(q1).T*trotx(-pi)*transl(0,0.25,-0.25);
             pan2updatedPoints = [pan2plotpose * [pan2Verts,ones(pan2VertexCount,1)]']';
             pan2Mesh_h.Vertices = pan2updatedPoints(:,1:3);
             drawnow();
             stored_i = i;
             if estop_state == 1
                 state = 0;
             end
             if i == 30
                 state = 5;
                 stored_i = 0;
             end
          end
       end
    end
    
    %% STATE 5: POUR MEAT FROM PAN 2 INTO PAN 1
    if state == 5
         disp('STATE 5')
         stored_state = 5;
         pan2plotpose = transl(-1.4,-0.5,1.325)*trotz(pi/2);
         qCurrent1 = r1.model.getpos();
         qEnd1 = r1.model.ikine(pan2plotpose*transl(0,-0.25,0.25)*trotx(pi),'q0', qCurrent1);
         JTrajQMatrix1 = jtraj(qCurrent1, qEnd1, 30);
         panplotpose = transl(-1.5,0.05,1.4)*trotx(-pi/8);
         qCurrent2 = [-1.2500, 1.1645, -1.4346,  0.9827, -0.0000, 1.1188,  0.4063];
         qEnd2 = r2.model.ikine(panplotpose*transl(0,0.2,0.15)*trotx(pi));
         JTrajQMatrix2 = jtraj(qCurrent2, qEnd2, 5);
         JTrajQMatrix2 = [JTrajQMatrix2;flipud(JTrajQMatrix2);JTrajQMatrix2;flipud(JTrajQMatrix2);JTrajQMatrix2;flipud(JTrajQMatrix2)];
         for i = (stored_i + 1):30
               if state == 5
                     r1.model.animate(JTrajQMatrix1(i,:));
                     q1 = r1.model.getpos();
                     finger1.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
                     finger2.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
                     finger1plotpose = finger1.fkine(-30/30*pi/2.2).T*transl(-0.5,0,0);
                     finger1plotpose = finger1plotpose*transl(-0.03,-0.005,0);
                     finger1updatedPoints = [finger1plotpose * [finger1Verts,ones(finger1VertexCount,1)]']';
                     finger1Mesh_h.Vertices = finger1updatedPoints(:,1:3);
                     drawnow();
                     finger1plotpose = finger1plotpose*transl(0.03,0.005,0);
                     finger2plotpose = finger2.fkine(30/30*pi/2.2).T*transl(0.5,0,0);
                     finger2plotpose = finger2plotpose*transl(0.03,-0.005,0);
                     finger2updatedPoints = [finger2plotpose * [finger2Verts,ones(finger2VertexCount,1)]']';
                     finger2Mesh_h.Vertices = finger2updatedPoints(:,1:3);
                     drawnow();    
                     finger2plotpose = finger2plotpose*transl(-0.03,0.005,0);
                     pan2plotpose = r1.model.fkine(q1).T*trotx(-pi)*transl(0,0.25,-0.25);
                     pan2updatedPoints = [pan2plotpose * [pan2Verts,ones(pan2VertexCount,1)]']';
                     pan2Mesh_h.Vertices = pan2updatedPoints(:,1:3);
                     drawnow();
                     r2.model.animate(JTrajQMatrix2(i,:));                
                     q2 = r2.model.getpos();
                     finger3.base = r2.model.fkine(q2).T*transl(0,0,0.023)*trotx(pi/2);
                     finger4.base = r2.model.fkine(q2).T*transl(0,0,0.023)*trotx(pi/2);
                     finger3plotpose = finger3.fkine(-30/30*pi/2.2).T*transl(-0.5,0,0);
                     finger3plotpose = finger3plotpose*transl(-0.03,-0.005,0);
                     finger3updatedPoints = [finger3plotpose * [finger3Verts,ones(finger3VertexCount,1)]']';
                     finger3Mesh_h.Vertices = finger3updatedPoints(:,1:3);
                     drawnow();
                     finger3plotpose = finger3plotpose*transl(0.03,0.005,0);
                     finger4plotpose = finger4.fkine(30/30*pi/2.2).T*transl(0.5,0,0);
                     finger4plotpose = finger4plotpose*transl(0.03,-0.005,0);
                     finger4updatedPoints = [finger4plotpose * [finger4Verts,ones(finger4VertexCount,1)]']';
                     finger4Mesh_h.Vertices = finger4updatedPoints(:,1:3);
                     drawnow();
                     finger4plotpose = finger4plotpose*transl(-0.03,0.005,0);
                     panplotpose = r2.model.fkine(q2).T*trotx(-pi)*transl(0,-0.20,-0.15);
                     panupdatedPoints = [panplotpose * [panVerts,ones(panVertexCount,1)]']';
                     panMesh_h.Vertices = panupdatedPoints(:,1:3);
                     drawnow();
                     stored_i = i;
                     if estop_state == 1
                         state = 0;
                     end
                     if i == 30                   
                         state = 6;
                         stored_i = 0;
                     end
                end
         end
    end
    %% STATE 6: KEEP FLIPPING THE PAN
    if state == 6
         disp('STATE 6')
         stored_state = 6;
         for i = (stored_i + 1):30
             if state == 6
                     r2.model.animate(JTrajQMatrix2(i,:));
                     q2 = r2.model.getpos();
                     finger3.base = r2.model.fkine(q2).T*transl(0,0,0.023)*trotx(pi/2);
                     finger4.base = r2.model.fkine(q2).T*transl(0,0,0.023)*trotx(pi/2);
                     finger3plotpose = finger3.fkine(-30/30*pi/2.2).T*transl(-0.5,0,0);
                     finger3plotpose = finger3plotpose*transl(-0.03,-0.005,0);
                     finger3updatedPoints = [finger3plotpose * [finger3Verts,ones(finger3VertexCount,1)]']';
                     finger3Mesh_h.Vertices = finger3updatedPoints(:,1:3);
                     drawnow();
                     finger3plotpose = finger3plotpose*transl(0.03,0.005,0);
                     finger4plotpose = finger4.fkine(30/30*pi/2.2).T*transl(0.5,0,0);
                     finger4plotpose = finger4plotpose*transl(0.03,-0.005,0);
                     finger4updatedPoints = [finger4plotpose * [finger4Verts,ones(finger4VertexCount,1)]']';
                     finger4Mesh_h.Vertices = finger4updatedPoints(:,1:3);
                     drawnow();
                     finger4plotpose = finger4plotpose*transl(-0.03,0.005,0);
                     panplotpose = r2.model.fkine(q2).T*trotx(-pi)*transl(0,-0.20,-0.15);
                     panupdatedPoints = [panplotpose * [panVerts,ones(panVertexCount,1)]']';
                     panMesh_h.Vertices = panupdatedPoints(:,1:3);
                     drawnow();
                     stored_i = i;
                     if estop_state == 1
                         state = 0;
                     end
                     if i == 30
                         state = 7;
                         stored_i = 0;
                     end
             end
         end
    end

    %% STATE 7: RMRC
    if state == 7
         disp('STATE 7')
         stored_state = 7;
         t = 10;             % Total time (s)
         deltaT = 0.05;      % Control frequency
         steps = t/deltaT;   % No. of steps for simulation
         epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
         W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
        
         % 1.2) Allocate array data
         m = zeros(steps,1);             % Array for Measure of Manipulability
         qMatrix = zeros(steps,6);       % Array for joint anglesR
         qdot = zeros(steps,6);          % Array for joint velocities
         theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
         x_rmrc = zeros(3,steps);             % Array for x-y-z trajectory
        
         % 1.3) Set up trajectory, initial pose
         s = lspb(0,1,steps); 
         for i=1:steps
             x_rmrc(1,i) = (1-s(i))*-1.15 + s(i)*-1.694; % Points in x
             x_rmrc(2,i) = (1-s(i))*-0.5 + s(i)*-0.7423; % Points in y
             x_rmrc(3,i) = (1-s(i))*1.575 + s(i)*1.923; % Points in z
             theta(1,i) = 0;                 % Roll angle 
             theta(2,i) = 1;                 % Pitch angle
             theta(3,i) = 0;                 % Yaw angle
         end
         
         T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x_rmrc(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
         q0 = [pi,0,0,0,0,0];                                                        % Initial guess for joint angles
         qMatrix(1,:) = r1.model.ikcon(T,q0);

         for i = 1:steps-1
            T = r1.model.fkine(qMatrix(i,:)).T;                                   % Get forward transformation at current joint state
            deltaX = x_rmrc(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
            Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
            Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
            Rdot = (1/deltaT)*(Rd - Rd);                                             % Calculate rotation matrix error
            S = Rdot*Ra';                                                           % Skew symmetric!
            linear_velocity = (1/deltaT)*deltaX;
            angular_velocity = [S(3,2);S(1,3);S(2,1)];                               % Check the structure of Skew Symmetric matrix!!
            xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
            J = r1.model.jacob0(qMatrix(i,:));                                    % Get Jacobian at current joint state
            m(i) = sqrt(det(J*J'));
            if m(i) < epsilon                                                       % If manipulability is less than given threshold
                lambda = (1 - m(i)/epsilon)*5E-2;
            else
                lambda = 0;
            end
            invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
            qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation
            for j = 1:6                                                             % Loop through joints 1 to 6
                if qMatrix(i,j) + deltaT*qdot(i,j) < r1.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                    qdot(i,j) = 0; % Stop the motor
                elseif qMatrix(i,j) + deltaT*qdot(i,j) > r1.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                    qdot(i,j) = 0; % Stop the motor
                end
            end
            qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
         end
         tic
         plot3(x_rmrc(1,:),x_rmrc(2,:),x_rmrc(3,:),'k.','LineWidth',1)
         for i = 1:steps
            r1.model.animate(qMatrix(i,:))
            q1 = r1.model.getpos();
            %plot3(r1.model.fkine(q1).t(1),r1.model.fkine(q1).t(2),r1.model.fkine(q1).t(3),'r.')
            finger1.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
            finger2.base = r1.model.fkine(q1).T*transl(0,0,0.023)*trotx(pi/2);
            finger1plotpose = finger1.fkine(-30/30*pi/2.2).T*transl(-0.5,0,0);
            finger1plotpose = finger1plotpose*transl(-0.03,-0.005,0);
            finger1updatedPoints = [finger1plotpose * [finger1Verts,ones(finger1VertexCount,1)]']';
            finger1Mesh_h.Vertices = finger1updatedPoints(:,1:3);
            drawnow();
            finger1plotpose = finger1plotpose*transl(0.03,0.005,0);
            finger2plotpose = finger2.fkine(30/30*pi/2.2).T*transl(0.5,0,0);
            finger2plotpose = finger2plotpose*transl(0.03,-0.005,0);
            finger2updatedPoints = [finger2plotpose * [finger2Verts,ones(finger2VertexCount,1)]']';
            finger2Mesh_h.Vertices = finger2updatedPoints(:,1:3);
            drawnow();
            finger2plotpose = finger2plotpose*transl(-0.03,0.005,0);
            drawnow();
         end
         state = 0;
         stored_state = 1;
         stored_i = 0;
    end
end

