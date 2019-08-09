%% stifness matrix method
% Input
clc,
clear;
n=6; % number of members
I=[2 3 2 2 1 3]; % moment of inertia
L=[4 4 2 2 3 2]; % length in m
m=[1 2 3 4 5 6]; % member number
uu=6; % Number of unrestrained degree of freedom 
ur=9; % Number of restrained degree of freedom
dof=uu + ur
uul=[1 2 3 4 5 6]; % global labels of unrestrained dof
url=[7 8 9 10 11 12 13 14 15]; % global labels of restrained dof
l1=[7 4 10 5]; % global labels for member 1
l2=[4 3 11 13]; % global labels for member 2
l3=[3 2  5 6];
l4=[2 8 6 12];
l5=[2 1 13 15];
l6=[1 9 6 14];
l=[l1;l2;l3;l4;l5;l6];
Ktotal=zeros(15);
fem1=[0;0;0;0]; % Local fixed end moments of member 1
fem2=[40;-40;60;60]; % Local fixed end moments of member 2
fem3=[0;0;0;0];
fem4=[0;0;0;0];
fem5=[15;-15;20;20];
fem6=[0;0;0;0];
%% rotation coef for each member
rc1 =4.*I./L;
rc2 =2.*I./L;
%% stiffness matrix 4 by 4 
for i=1:n
    Knew=zeros (15);
    k1=[rc1(i); rc2(i); (rc1(i)+rc2(i))/L(i); (-(rc1(i)+rc2(i))/L(i))];
    k2=[rc2(i); rc1(i); (rc1(i)+rc2(i))/L(i); (-(rc1(i)+rc2(i))/L(i))];
    k3=[(rc1(i)+rc2(i))/L(i); (rc1(i)+rc2(i))/L(i); (2*(rc1(i)+rc2(i))/(L(i)^2));(-2*(rc1(i)+rc2(i))/(L(i)^2))];
    k4=-k3;
    K= [k1 k2 k3 k4];
    fprintf('Member Number = ');
    disp(i);
    fprintf('Local Stiffness matrix of member, [K]=\n');
    disp (K);
    for p=1:4
        for q=1:4
            Knew((l(i,p)),(l(i,q))) = K(p,q);
        end
    end
    Ktotal=Ktotal + Knew;
    if i==1
        Kg1=K;
    elseif i==2
        Kg2=K;
    elseif i==3
        Kg3=K;
     elseif i==4
        Kg4=K;
     elseif i==5
        Kg5=K;
     elseif i==6
        Kg6=K;
      end
end
fprintf('Stiffness Matrix of complete structure, [Ktotal]=\n');
disp (Ktotal);
Kunr=zeros(6);
for x=1:uu
    for y=1:uu
        Kunr(x,y)=Ktotal(x,y);
    end 
end
fprintf ('unrestrained Stiffness sub-matrix, [Kuu]=\n');
disp (Kunr);
KuuInv=inv(Kunr);
fprintf ('Inverse of Unrestrained Stiffness sub-matrix,[KuuInverse]=\n');
disp (KuuInv);
%% Creation of joint load vector
jl=[15;-15;40;-40;20;0;0;0;0;0;-60;0;-60;0;-20];  % values given in kN or kNm
jlu=[15;-15;40;-40;20;0];  % load vector in unrestrained dof
delu=KuuInv*jlu;
fprintf('Joint Load Vector,[J1]= \n');
disp (jl);
fprintf('Unrestrained displacements, [DelU]=\n');
disp (delu);
delr=[0;0;0;0;0;0;0;0;0];
del=zeros(6,1);
del=[delu;delr];
deli=zeros(4,1);
for i=1:n
    for p=1:4
        deli(p,1)=del ((l(i,p)),1);
    end
    if i==1
        delbar1=deli;
        mbar1=(Kg1*delbar1)+fem1;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacement matrix [DeltaBar]=\n');
        disp(delbar1);
        fprintf('Global End moment matrix [Mbar]=\n');
        disp (mbar1);
    elseif i==2
        delbar2=deli;
        mbar2=(Kg2*delbar2)+fem2;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacement matrix [DeltaBar]=\n');
        disp(delbar2);
        fprintf('Global End moment matrix [Mbar]=\n');
        disp (mbar2);
     elseif i==3
        delbar3=deli;
        mbar3=(Kg3*delbar3)+fem3;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacement matrix [DeltaBar]=\n');
        disp(delbar3);
        fprintf('Global End moment matrix [Mbar]=\n');
        disp (mbar3);
      elseif i==4
        delbar4=deli;
        mbar4=(Kg4*delbar4)+fem4;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacement matrix [DeltaBar]=\n');
        disp(delbar4);
        fprintf('Global End moment matrix [Mbar]=\n');
        disp (mbar4);
     elseif i==5
        delbar5=deli;
        mbar5=(Kg5*delbar5)+fem5;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacement matrix [DeltaBar]=\n');
        disp(delbar5);
        fprintf('Global End moment matrix [Mbar]=\n');
        disp (mbar5);
      elseif i==6
        delbar6=deli;
        mbar6=(Kg6*delbar6)+fem6;
        fprintf('Member Number=');
        disp(i);
        fprintf('Global displacement matrix [DeltaBar]=\n');
        disp(delbar6);
        fprintf('Global End moment matrix [Mbar]=\n');
        disp (mbar6);
    end
end





        
    


