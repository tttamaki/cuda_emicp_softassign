
rand("seed", fscanfMat("seed.txt"));

R1 = rand(3,3);
R2 = R1 + eye(3,3)*0.01;
[U,S,V]=svd(R2);
H = eye(3,3);
H(3,3)=det(U)*det(V);
R = U*H*V';
t=rand(3,1)*0.2;

fprintfMat("RT.txt", [R'; (-R'*t)']);

P=fscanfMat("P101.txt");
P(:,4)=[]
Q=R * P' + t * ones(1, 101);
Q=Q';
fprintfMat("Qnew101.txt", Q);

P=fscanfMat("P504.txt");
P(:,4)=[]
Q=R * P' + t * ones(1, 504);
Q=Q';
fprintfMat("Qnew504.txt", Q);

P=fscanfMat("P1007.txt");
P(:,4)=[]
Q=R * P' + t * ones(1, 1007);
Q=Q';
fprintfMat("Qnew1007.txt", Q);

P=fscanfMat("P3097.txt");
P(:,4)=[]
Q=R * P' + t * ones(1, 3097);
Q=Q';
fprintfMat("Qnew3097.txt", Q);

P=fscanfMat("P5032.txt");
P(:,4)=[]
Q=R * P' + t * ones(1, 5032);
Q=Q';
fprintfMat("Qnew5032.txt", Q);

P=fscanfMat("P10064.txt");
P(:,4)=[]
Q=R * P' + t * ones(1, 10064);
Q=Q';
fprintfMat("Qnew10064.txt", Q);



exit;

