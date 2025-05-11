function RRR3D(th1,th2,th3)
%% A function named RRR3D used to generate the DH model of the RRR robot in 3D space %%
% INPUT(S):  th1, th2, th3: The three revolute angles     
% OUTPUT(S): NIL- But outputs can be added if required by user
%%
    % Initialization of coordinates %%
    O= [0;0;0];
    A = O+ [0;0;45];
    E = O+ [0;0;25];
    F = O+ [0;0;15];
    G = O+ [0;0;20];

%% Link-1 (blue) and first revolute joint (blue) %%
    
    % Construction of link from origin to base of first revolute joint %
    
    plot3([O(1),A(1)],[O(2),A(2)],[O(3),A(3)],'color','b','LineWidth',2)
    hold on;
    scatter3(0,0,0,'MarkerEdgeColor','k','MarkerFaceColor','k','Linewidth',1)
    scatter3(A(1),A(2),A(3),'MarkerEdgeColor','k','MarkerFaceColor','b','Linewidth',1)
    %title('RRR robot performing the dental scan- Task space')
    title('RRR robot performing the 3D printing- Task space')
    % Plot parameters %
    
    xlim([-80,120]);
    ylim([-80,120]);
    zlim([0,120]);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on;
    grid minor;
    set(gcf,'color','white')
    set(gca,'FontSize',20,'FontName','Times New Roman','FontWeight','Bold');
    x0 = 25; y0 = 45; % Origin for the plot screen
    largeur =950; % Length of plot screen from origin
    hauteur =550; % Width of plot screen from origin
    set(gcf,'units','points','position',[ x0, y0, largeur, hauteur])
    
    % Construction of first revolute joint (blue) using function 'cylinder' %
    
    theta= 0:0.1:2*pi;
    r= 2;
    x1= r.*cos(theta);y1= r.*sin(theta);
    z1= zeros(1, length(x1));
    z1= z1+10;
    z2= z1+15;
    fill3(x1,y1,z1,'b');
    fill3(x1,y1,z2,'b');
    for j= 1:length(x1)
        plot3([x1(1,j),x1(1,j)],[y1(1,j),y1(1,j)],[z1(1,j),z2(1,j)],'b','LineWidth',2);
    end
    
%% Link twist and second revolute joint %%
    
    % Rotation matrices for link-twist and revolution caused by th1 %
    % Sub-function rotations called to apply rotation/twist on corresponding axis%
    
    [rx1,~,~]= rotations(-pi/2);
    [~,~,rz1]= rotations(th1);
    
    Aux1= [0;0;5];
    Aux2= [0;0;20];
    Aux1= rz1*rx1*Aux1;
    Aux2= rz1*rx1*Aux2;
    
    %Plot for link twist %
   
    B= A+Aux1;
    C= A+Aux2;

    % Plot of link A-B %

    plot3([B(1),A(1)],[B(2),A(2)],[B(3),A(3)],'color','b','LineWidth',2)

    %Construction of second revolute joint (red) using function 'cylinder' %
    
    cylinderplot (theta,B,C,r,rz1*rx1,'r'); % Calling sub-function cylinderplot to create cylinder %
    
%% Link-2 (blue) and third revolute joint (magenta) %%
    
    % Sub-function rotations called to apply rotation/twist on corresponding axis%
    
    [~,ry1,~]= rotations(-pi/2);
    [~,~,rz2]= rotations(th2);

    % Creation of link D-E from second revolute joint%
    
    D= rz1*rx1*rz2*ry1*O;
    E= rz1*rx1*rz2*ry1*E;
    D= C+D;
    E= C+E;
    plot3([E(1),D(1)],[E(2),D(2)],[E(3),D(3)],'color','r','LineWidth',2)
    
    %Construction of third revolute joint (magenta) using function 'cylinder' %
    
    [~,ry2,~]= rotations(-pi/2);
    F= rz1*rx1*rz2*ry1*ry2*F;
    Erot= rz1*rx1*rz2*ry1*ry2*O;

    F= E+F;
    E= E+Erot;
    
    cylinderplot (theta,E,F,r,rz1*rx1*rz2*ry1*ry2,'m'); % Calling sub-function cylinderplot to create cylinder %

%% Link-3 and end-effector (magenta) %%
    
    % Sub-function rotations called to apply rotation/twist on corresponding axis%

    [~,ry3,~]= rotations(-pi/2);
    [~,~,rz3]= rotations(th3);

    Frot= rz1*rx1*rz2*ry1*ry2*rz3*ry3*O;
    G= rz1*rx1*rz2*ry1*ry2*rz3*ry3*G;
    
    G= F+G;
    F= F+Frot;
    
    % Creation of link F-G & end-effector from third revolute joint%

    plot3([F(1),G(1)],[F(2),G(2)],[F(3),G(3)],'color','m','LineWidth',2);
    scatter3(G(1),G(2),G(3),'MarkerEdgeColor','k','MarkerFaceColor','m','Linewidth',1);
    
    hold off;

%% SUB-FUNCTION(S)%%

%%
    function [rx,ry,rz]= rotations(th)
    %% A SUB-FUNCTION "rotations" TO APPLY ROTATIONS/LINK TWIST TO ROBOT %% 
    % INPUT(S)  : th: Rotation angle
    % OUTPUT(S) : rx,ry,rz: The x,y,z rotations matrices
    %%
        rx= [1 0 0; 0 cos(th) -sin(th);0 sin(th) cos(th)];
        ry= [cos(th) 0 -sin(th);0 1 0;-sin(th) 0 cos(th)];
        rz= [cos(th) -sin(th) 0;sin(th) cos(th) 0; 0 0 1];
    end
%%  
    function cylinderplot (theta,P1,P2,r,Rot,couleur)
    %% A SUB-FUNCTION "cylinderplot" TO CREATE REVOLUTE JOINT(S) %% 
    % INPUT(S)  : theta: For creation of top & bottom circle of revolute
    %             P1,P2: Coordinates of top & bottom of revolute
    %             r: Radius of the cylinder/revolute
    %             Rot: Rotation angles to the joint from robot architecture
    %             couleur: A string which defines the color of the joint
    % OUTPUT(S) : NIL
    %%
        x1= r.*cos(theta);y1= r.*sin(theta);
        z1= zeros(1, length(x1));
        BE(1,:)= x1;BE(2,:)= y1;BE(3,:)= z1;
        BE= Rot*BE;
        Bott(1,:) = BE(1,:) + P1(1);
        Bott(2,:) = BE(2,:) + P1(2);
        Bott(3,:) = BE(3,:) + P1(3);
        Top(1,:) = BE(1,:) + P2(1);
        Top(2,:) = BE(2,:) + P2(2);
        Top(3,:) = BE(3,:) + P2(3);
    
        for j= 1:length(Bott)
            plot3([Bott(1,j),Top(1,j)],[Bott(2,j),Top(2,j)],[Bott(3,j),Top(3,j)],couleur,'LineWidth',2);
        end
    
        fill3(Bott(1,:),Bott(2,:),Bott(3,:),couleur);
        fill3(Top(1,:),Top(2,:),Top(3,:),couleur);
    
    end

end