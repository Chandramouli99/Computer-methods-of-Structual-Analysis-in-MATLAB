%% stifness matrix method
% Input
clc,
clear;
n=2; % number of members
I=[1 1]; % moment of inertia
L=[6 6]; % length in m
m=[1 2]; % member number
uu=1; % Number of unrestrained degree of freedom 
ur=5; % Number of restrained degree of freedom 
uul=[1]; % global labels of unrestrained dof
url=[2 3 4 5 6]; % global labels of restrained dof
l1=[2 1 4 5]; % global labels for member 1
l2=[1 3 5 6]; % global labels for member 2
l=[l1;l2];
Ktotal=zeros (6);
fem1=[60;-60;60;60]; % Local fixed end moments of member 1
fem2=[60;-60;60;60]; % Local fixed end moments of member 2
%% rotation coef for each member
rc1 =4.*I./L;
rc2 =2.*I./L;
%% stiffness matrix 4 by 4 
for i=1:n
    Knew=zeros (6);
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
    end
end
fprintf('Stiffness Matrix of complete structure, [Ktotal]=\n');
disp (Ktotal);
Kunr=zeros(1);
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
jl=[-120;-60;60;-60;-120;-60];  % values given in kN or kNm
jlu=[-120;-60];  % load vector in unrestrained dof
delu=KuuInv*jlu;
fprintf('Joint Load Vector,[J1]= \n');
disp (jl);
fprintf('Unrestrained displacements, [DelU]=\n');
disp (delu);
delr=[0;0;0;0];
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
    end
end





        
    


