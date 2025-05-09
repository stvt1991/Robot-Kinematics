function pl = Scara3D(L1,L2,theta1, theta2, rho, theta4)
%% Creator: Swaminath Venkateswaran, ESILV Engineering School, Paris, France %%
%% A function named Scara3D used to generate the DH model of the SCARA robot in 3D space %%
% INPUT(S):  L1,L2: Length of the two links
%            theta1, theta2, theta4: The three revolute angles
%            rho: The prismatc stroke length
% OUTPUT(S): pl: A vector which saves all plots

%% Origin of the robot %%

    Ori= [0,0,0];
    Rev1= [0,0,10]; %Base of first revolute joint
    
    %Plotting the link from origin to base of revolute joint-1 %
    h1=plot3([Ori(1,1),Rev1(1,1)],[Ori(1,2),Rev1(1,2)],[Ori(1,3),Rev1(1,3)],'k','LineWidth',2);
    hold on;
    grid on;
    % Plot parameters %
    set(gca,'FontSize',20,'FontName','Times New Roman','FontWeight','Bold');
    title('The SCARA robot')
    x0 = 10; y0 = 10; % Origin for the plot screen
    largeur =1150; % Length of plot screen from origin
    hauteur =650; % Width of plot screen from origin
    set(gcf,'units','points','position',[ x0, y0, largeur, hauteur])
    xlabel('x');
    ylabel('y');
    zlabel('z');
    set(gcf,'color','white')
    %A scatter point to highlight the robot base at x0,y0,z0%
    h2=scatter3(0,0,0,'MarkerEdgeColor','k','MarkerFaceColor','k','Linewidth',2);
    
    %% Construction of the first revolute joint %%
        
    % Cylinder to construct the red colored revolute joint-1%
    r = 4; %Radius
    h=10; %Height
    [x,y,z] = cylinder(r);
    O= [0,0,10]; % Origin of cylinder (Revolute joint)%
    x=x+O(1);
    z=z*h+O(3); %Extrusion of cylinder%
    y=y+O(2);
    H1=surf(x,y,z); % Generating cylinder%
    xlim([-150,200]);
    ylim([-150,200]);
    zlim([-10,70]);
    set(H1,'facecolor','r','edgecolor','r');
    
    % To close the top and bottom faces of the revolute joint % !! Least priority
    theta= 0:0.1:2*pi;
    x1= Ori(1)+r.*cos(theta);y1= Ori(2)+r.*sin(theta);
    z1= Ori(3)*ones(1,length(x1));
    h3=fill3(x1,y1,z1,'r');
    theta= 0:0.1:2*pi;
    x1= Ori(1)+r.*cos(theta);y1= Ori(2)+r.*sin(theta);
    z1= (h+Ori(3))*ones(1,length(x1));
    h4=fill3(x1,y1,z1,'r');

    % Note that, the top face of the first revolute joint is the
    % position of the frame x1,y1,z1
    
    %% Construction of the link L1 %%

    L11= Rev1 + [0,0,h];
    L12= L11 + [0,0,10];
    h5=plot3([L11(1,1),L12(1,1)],[L11(1,2),L12(1,2)],[L11(1,3),L12(1,3)],'b','LineWidth',2);
    
    %% Robot movement sequences %%
    % Application of rotation angle caused by theta-1 on Link-1% 

    A1= L12(1,1)+L1*cos(theta1);
    B1= L12(1,2)+L1*sin(theta1);
    L13= [A1,B1,L12(3)];
    h6=plot3([L13(1,1),L12(1,1)],[L13(1,2),L12(1,2)],[L13(1,3),L12(1,3)],'b','LineWidth',2);
    L14= L13+ [0,0,5];
    h7=plot3([L13(1,1),L14(1,1)],[L13(1,2),L14(1,2)],[L13(1,3),L14(1,3)],'b','LineWidth',2);
    

    %% Construction of the second revolute joint %%
    
    [x,y,z] = cylinder(r);
    O= L14; %Origin of second cylinder at the tip of link-1%
    x=x+O(1);
    z=z*h+O(3); %Extrusion of cylinder
    y=y+O(2);
    H2=surf(x,y,z);
    set(H2,'facecolor','m','edgecolor','m');

    % To close the top and bottom faces of the revolute joint % !! Least priority
    theta= 0:0.1:2*pi;
    x1= L14(1)+r.*cos(theta);y1= L14(2)+r.*sin(theta);
    z1= L14(3)*ones(1,length(x1));
    h8=fill3(x1,y1,z1,'m');
    theta= 0:0.1:2*pi;
    x1= L14(1)+r.*cos(theta);y1= L14(2)+r.*sin(theta);
    z1= [h+L14(3)]*ones(1,length(x1));
    h9=fill3(x1,y1,z1,'m');

    % Note that, the top face of the second revolute joint is the
    % position of the frame x2,y2,z2

    %% Construction of the link L2 %%
    
    L21= L14 + [0,0,20];
    h10=plot3([L21(1,1),L21(1,1)],[L21(1,2),L21(1,2)],[h+L14(3),L21(1,3)],'color',[1.00,0.41,0.16],'LineWidth',2);
     
    A2= L21(1)+L2*cos(theta1+theta2);
    B2= L21(2)+L2*sin(theta1+theta2);
    L22= [A2,B2,L21(3)];
    h11=plot3([L21(1,1),L22(1,1)],[L21(1,2),L22(1,2)],[L21(1,3),L22(1,3)],'color',[1.00,0.41,0.16],'LineWidth',2);
    h12=scatter3(L22(1),L22(2),L22(3),'MarkerEdgeColor',[1.00,0.41,0.16],'MarkerFaceColor',[1.00,0.41,0.16],'Linewidth',1);
    
    %% Generation of the prismatic joint %%
    L23= [L22(1),L22(2),L22(3)-rho]; % This link varies according to the prismatic stroke
    h13=plot3([L22(1,1),L23(1,1)],[L22(1,2),L23(1,2)],[L22(1,3),L23(1,3)],'color',[1.00,0.41,0.16],'LineWidth',2);
    Blocksize = 5;
    rects(theta4,L22,rho,Blocksize); % Sub-function rects called to generate the cuboid
    
    %% Generation of the third link to the final revloute joint %%
    L31= [L23(1),L23(2),L23(3)-Blocksize];
    L32= [L31(1),L31(2),L31(3)-10];
    h14=plot3([L31(1,1),L32(1,1)],[L31(1,2),L32(1,2)],[L31(1,3),L32(1,3)],'color',[0.07,0.62,1.00],'LineWidth',2);
   
    %% Generation of the third revolute joint %%

    [x,y,z] = cylinder(r);
    O= L32;
    x=x+O(1);
    z=-z*h+O(3); %Extrusion in the negative-z
    y=y+O(2);
    H3=surf(x,y,z);
    set(H3,'facecolor',[0.47,0.67,0.19],'edgecolor',[0.47,0.67,0.19]);
    
    %% The end-effector %%
    E= L32+[0,0,-10];
    EF=scatter3(E(1),E(2),E(3),'MarkerEdgeColor','k','MarkerFaceColor','k','Linewidth',5);
    pl= [h1,h2,h3,h4,h5,h6,h7,h8,h9,h10,h11,h12,h13,h14,H1,H2,H3];
    hold off;

%% A SUB-FUNCTION "rects" TO CONSTRUCT THE PRISMATIC CUBOID %% !! Least priority
% INPUT(S)  : theta4, L22, rho, bl(depth of block)
% OUTPUT(S) : NIL
%%

    function rects(theta4,L22,rho,bl)

        % Rot-Z matrix for cuboid as it rotates according to theta-4%
        Rotaz= [cos(theta4),sin(theta4);-sin(theta4),cos(theta4)];
        
        % A cuboid of size 20*10*5 % 
        % Generation of rectangle for bottom & top faces %
        Coord1= [L22(1);L22(2)]+Rotaz*[-10;-5];
        Coord2= [L22(1);L22(2)]+Rotaz*[10;-5];
        Coord3= [L22(1);L22(2)]+Rotaz*[10;5];
        Coord4= [L22(1);L22(2)]+Rotaz*[-10;5];

        %Extracting phases of cuboid %
        Rect1x= [Coord1(1,1),Coord2(1,1),Coord3(1,1),Coord4(1,1)];
        Rect1y= [Coord1(2,1),Coord2(2,1),Coord3(2,1),Coord4(2,1)];
        Rect1z= [L22(3)-rho,L22(3)-rho,L22(3)-rho,L22(3)-rho];
        Rect2z= [L22(3)-rho-bl,L22(3)-rho-bl,L22(3)-rho-bl,L22(3)-rho-bl]; %Depth of cuboid of 5%
        
        %Extraction of sides to fill the cuboid%
        Side1x= [Coord1(1,1),Coord1(1,1),Coord4(1,1),Coord4(1,1)];
        Side1y= [Coord1(2,1),Coord1(2,1),Coord4(2,1),Coord4(2,1)];
        Side1z= [L22(3)-rho,L22(3)-rho-bl,L22(3)-rho-bl,L22(3)-rho];
        Side2x= [Coord2(1,1),Coord2(1,1),Coord3(1,1),Coord3(1,1)];
        Side2y= [Coord2(2,1),Coord2(2,1),Coord3(2,1),Coord3(2,1)];
        Side2z= [L22(3)-rho,L22(3)-rho-bl,L22(3)-rho-bl,L22(3)-rho];
        Side3x= [Coord1(1,1),Coord2(1,1),Coord2(1,1),Coord1(1,1)];
        Side3y= [Coord1(2,1),Coord2(2,1),Coord2(2,1),Coord1(2,1)];
        Side3z= [L22(3)-rho,L22(3)-rho,L22(3)-rho-bl,L22(3)-rho-bl];
        Side4x= [Coord4(1,1),Coord3(1,1),Coord3(1,1),Coord4(1,1)];
        Side4y= [Coord4(2,1),Coord3(2,1),Coord3(2,1),Coord4(2,1)];
        Side4z= [L22(3)-rho,L22(3)-rho,L22(3)-rho-bl,L22(3)-rho-bl];

        %Filling all faces of cuboid%
        h100=fill3(Rect1x,Rect1y,Rect1z,'r');
        h101=fill3(Rect1x,Rect1y,Rect2z,'r');
        h102=fill3(Side1x,Side1y,Side1z,'r');
        h103=fill3(Side2x,Side2y,Side2z,'r');
        h104=fill3(Side3x,Side3y,Side3z,'r');
        h105=fill3(Side4x,Side4y,Side4z,'r');

    end


end
