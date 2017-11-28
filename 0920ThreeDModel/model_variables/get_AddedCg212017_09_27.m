function out1 = get_AddedCg212017_09_27(in1,in2,in3)
%GET_ADDEDCG212017_09_27
%    OUT1 = GET_ADDEDCG212017_09_27(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    27-Sep-2017 17:04:44

AddedMmgx = in3(1,:);
AddedMmgy = in3(2,:);
AddedMmgz = in3(3,:);
dphif = in2(4,:);
dphit = in2(7,:);
dpsif = in2(6,:);
dthetaf = in2(5,:);
dthetat = in2(8,:);
phif = in1(4,:);
phit = in1(7,:);
psif = in1(6,:);
thetaf = in1(5,:);
thetat = in1(8,:);
t2 = phif-phit;
t3 = thetaf-thetat;
t4 = cos(psif);
t9 = pi.*(1.0./2.0);
t5 = -t9+thetaf;
t6 = cos(t5);
t7 = cos(t2);
t8 = sin(t3);
t10 = cos(t3);
t11 = sin(t2);
t12 = cos(phif);
t13 = sin(t5);
t14 = sin(phif);
t15 = sin(psif);
t16 = t14.*t15;
t17 = t4.*t6.*t12;
t18 = t16+t17;
t19 = t12.*t15;
t47 = t4.*t6.*t14;
t20 = t19-t47;
t21 = t7.*t10.*t18.*4.0;
t22 = t4.*t10.*t11.*t13.*4.0;
t23 = t7.*t10.*t18.*8.0;
t24 = t4.*t10.*t11.*t13.*8.0;
t25 = t4.*t14;
t29 = t6.*t12.*t15;
t26 = t25-t29;
t27 = t8.*t11.*t18.*4.0;
t28 = t8.*t11.*t18.*8.0;
t30 = t4.*t12;
t31 = t6.*t14.*t15;
t32 = t30+t31;
t33 = t7.*t10.*t26.*4.0;
t34 = t7.*t10.*t26.*8.0;
t35 = t8.*t11.*t26.*4.0;
t36 = t7.*t8.*t13.*t15.*4.0;
t37 = t8.*t11.*t26.*8.0;
t38 = t7.*t8.*t13.*t15.*8.0;
t39 = t6.*t10.*t11.*4.0;
t40 = t6.*t10.*t11.*8.0;
t41 = t6.*t7.*t8.*4.0;
t42 = t8.*t11.*t12.*t13.*4.0;
t43 = t6.*t7.*t8.*8.0;
t44 = t8.*t11.*t12.*t13.*8.0;
t45 = t10.*t11.*t18.*4.0;
t54 = t4.*t7.*t10.*t13.*4.0;
t46 = t45-t54;
t48 = t4.*t6.*t7.*t10.*4.0;
t49 = t4.*t10.*t11.*t12.*t13.*4.0;
t50 = t7.*t10.*t13.*4.0;
t51 = t10.*t11.*t26.*4.0;
t52 = t7.*t10.*t13.*t15.*4.0;
t53 = t51+t52;
t55 = t10.*t11.*t20.*4.0;
t56 = t21+t22+t55;
t57 = t10.*t11.*t32.*4.0;
t67 = t10.*t11.*t13.*t15.*4.0;
t58 = t33+t57-t67;
t59 = t6.*t7.*t10.*4.0;
t60 = t10.*t11.*t12.*t13.*4.0;
t61 = t59+t60;
t62 = t10.*t11.*t13.*t14.*4.0;
t86 = t7.*t10.*t12.*t13.*4.0;
t63 = t39+t62-t86;
t64 = t10.*t11.*t13.*t14.*8.0;
t65 = t10.*t11.*t20.*8.0;
t66 = t10.*t11.*t32.*8.0;
t68 = t7.*t8.*t13.*t14.*4.0;
t69 = t10.*t26.*4.0;
t70 = t7.*t8.*t32.*4.0;
t71 = t10.*t18.*4.0;
t72 = t7.*t8.*t20.*4.0;
t73 = t4.*t8.*t12.*t13.*4.0;
t74 = t4.*t7.*t8.*t13.*4.0;
t75 = t4.*t7.*t10.*t13.*t14.*4.0;
t87 = t6.*t10.*t11.*t12.*4.0;
t129 = t10.*t12.*t13.*4.0;
t149 = t6.*t8.*t12.*4.0;
t150 = t6.*t7.*t10.*t14.*4.0;
t76 = t41+t42+t50+t68-t87-t129-t149-t150;
t77 = AddedMmgz.*t61.*t76;
t78 = t8.*t12.*t13.*t15.*4.0;
t79 = t6.*t7.*t10.*t15.*4.0;
t80 = t7.*t10.*t13.*t14.*t15.*4.0;
t81 = t10.*t11.*t12.*t13.*t15.*4.0;
t82 = -t35-t36+t69+t70+t78+t79+t80+t81;
t83 = t8.*t20.*4.0;
t84 = t8.*t32.*4.0;
t85 = t8.*t13.*t14.*4.0;
t88 = t8.*t26.*4.0;
t145 = t7.*t10.*t32.*4.0;
t89 = t51+t52+t88-t145;
t90 = AddedMmgx.*t46.*t89;
t91 = t8.*t18.*4.0;
t146 = t7.*t10.*t20.*4.0;
t92 = t45-t54+t91-t146;
t93 = t90-AddedMmgy.*t53.*t92;
t94 = t10.*t32.*4.0;
t95 = t8.*t11.*t13.*t15.*4.0;
t96 = t10.*t20.*4.0;
t97 = t7.*t8.*t18.*4.0;
t98 = t4.*t8.*t13.*t14.*4.0;
t99 = t4.*t6.*t10.*t11.*4.0;
t100 = t4.*t8.*t11.*t13.*4.0;
t101 = t7.*t10.*t12.*t13.*t15.*4.0;
t109 = t7.*t8.*t26.*4.0;
t164 = t8.*t13.*t14.*t15.*4.0;
t165 = t6.*t10.*t11.*t15.*4.0;
t102 = t94+t95+t101-t109-t164-t165;
t103 = t4.*t6.*t8.*t11.*4.0;
t166 = t4.*t10.*t13.*t14.*4.0;
t167 = t4.*t7.*t8.*t12.*t13.*4.0;
t104 = t21+t22+t83+t103-t166-t167;
t105 = t4.*t6.*t8.*t11.*8.0;
t106 = t21+t22+t83;
t107 = t8.*t20.*8.0;
t108 = -t27+t71+t72+t74;
t110 = t4.*t7.*t8.*t13.*8.0;
t111 = -t96+t97+t100;
t112 = t33-t67+t84;
t113 = t35+t36-t69-t70;
t114 = t10.*t13.*t14.*t15.*4.0;
t115 = t7.*t8.*t12.*t13.*t15.*4.0;
t168 = t6.*t8.*t11.*t15.*4.0;
t116 = t33-t67+t84+t114+t115-t168;
t117 = t8.*t32.*8.0;
t118 = t10.*t13.*t14.*t15.*8.0;
t119 = t7.*t8.*t12.*t13.*t15.*8.0;
t120 = t35+t36;
t121 = AddedMmgy.*t120.*(1.0./2.0);
t122 = t37+t38;
t123 = t121-AddedMmgy.*t122.*(1.0./4.0);
t124 = t7.*t10.*t12.*t13.*8.0;
t125 = t41+t42;
t126 = AddedMmgz.*t125.*(1.0./2.0);
t127 = t43+t44;
t128 = t126-AddedMmgz.*t127.*(1.0./4.0);
t130 = t6.*t10.*t14.*4.0;
t131 = t8.*t11.*t13.*4.0;
t132 = t6.*t7.*t8.*t12.*4.0;
t133 = -t39-t85+t86+t130+t131+t132;
t134 = t6.*t10.*t14.*8.0;
t135 = t8.*t13.*t14.*8.0;
t136 = t8.*t11.*t13.*8.0;
t137 = t6.*t7.*t8.*t12.*8.0;
t138 = t22+t23+t65+t83;
t139 = t94+t95-t109;
t140 = t34+t66-t67+t84;
t141 = t10.*t13.*t14.*4.0;
t142 = t7.*t8.*t12.*t13.*4.0;
t148 = t6.*t8.*t11.*4.0;
t143 = t141+t142-t148;
t144 = t41+t42+t68-t129;
t147 = t27+t48+t49-t71-t72+t73-t74+t75;
t151 = t4.*t6.*t7.*t10.*t12.*4.0;
t152 = t21+t24+t83+t105+t151-t4.*t6.*t8.*t14.*4.0-t4.*t10.*t13.*t14.*8.0-t4.*t7.*t8.*t12.*t13.*8.0;
t153 = -t40+t124+t134-t135+t136+t137;
t154 = t6.*t8.*t14.*t15.*4.0;
t155 = t33+t84+t118+t119+t154-t6.*t8.*t11.*t15.*8.0-t10.*t11.*t13.*t15.*8.0-t6.*t7.*t10.*t12.*t15.*4.0;
t156 = t27+t48+t49-t74;
t157 = t35+t36-t79-t81;
t158 = t41+t42+t50-t87;
t159 = AddedMmgx.*t111.*t147;
t160 = AddedMmgy.*t82.*t139;
t161 = AddedMmgz.*t143.*(t41+t42+t50+t68-t87-t129-t149-t150);
t162 = t159+t160+t161;
t172 = t4.*t7.*t10.*t12.*t13.*4.0;
t163 = t96-t97+t98+t99-t100-t172;
t169 = AddedMmgx.*t89.*t111;
t170 = AddedMmgy.*t92.*t139;
t171 = t169+t170;
t173 = AddedMmgy.*t139.*t163;
t174 = AddedMmgx.*t102.*t111;
t175 = t173+t174;
out1 = reshape([dpsif.*(AddedMmgx.*t53.*(1.0./2.0)-AddedMmgx.*(t10.*t11.*t26.*8.0+t7.*t10.*t13.*t15.*8.0).*(1.0./4.0))+dthetaf.*(AddedMmgx.*(t28+t4.*t6.*t7.*t10.*8.0-t4.*t7.*t8.*t13.*8.0+t4.*t10.*t11.*t12.*t13.*8.0).*(1.0./4.0)-AddedMmgx.*(t27+t48+t49-t4.*t7.*t8.*t13.*4.0).*(1.0./2.0))+dthetat.*(AddedMmgx.*(t27-t4.*t7.*t8.*t13.*4.0).*(1.0./2.0)-AddedMmgx.*(t28-t4.*t7.*t8.*t13.*8.0).*(1.0./4.0))+dphif.*(AddedMmgx.*t56.*(1.0./2.0)-AddedMmgx.*(t23+t24+t65).*(1.0./4.0))-dphit.*(AddedMmgx.*(t21+t22).*(1.0./2.0)-AddedMmgx.*(t23+t24).*(1.0./4.0)),-dpsif.*(AddedMmgx.*t139.*(1.0./2.0)-AddedMmgx.*(t10.*t32.*8.0-t7.*t8.*t26.*8.0+t8.*t11.*t13.*t15.*8.0).*(1.0./4.0))+dphif.*(AddedMmgx.*t108.*(1.0./2.0)-AddedMmgx.*(-t28+t110+t10.*t18.*8.0+t7.*t8.*t20.*8.0).*(1.0./4.0))-dthetat.*(AddedMmgx.*t106.*(1.0./2.0)-AddedMmgx.*(t23+t24+t107).*(1.0./4.0))+dphit.*(AddedMmgx.*(t27-t74).*(1.0./2.0)-AddedMmgx.*(t28-t110).*(1.0./4.0))+dthetaf.*(AddedMmgx.*t104.*(1.0./2.0)-AddedMmgx.*(t23+t24+t105+t107-t4.*t10.*t13.*t14.*8.0-t4.*t7.*t8.*t12.*t13.*8.0).*(1.0./4.0)),0.0,0.0,0.0,0.0,dpsif.*(AddedMmgy.*t46.*(1.0./2.0)-AddedMmgy.*(t10.*t11.*t18.*8.0-t4.*t7.*t10.*t13.*8.0).*(1.0./4.0))-dthetat.*t123-dphif.*(AddedMmgy.*t58.*(1.0./2.0)-AddedMmgy.*(t34+t66-t10.*t11.*t13.*t15.*8.0).*(1.0./4.0))+dphit.*(AddedMmgy.*(t33-t10.*t11.*t13.*t15.*4.0).*(1.0./2.0)-AddedMmgy.*(t34-t10.*t11.*t13.*t15.*8.0).*(1.0./4.0))+dthetaf.*(AddedMmgy.*(t35+t36-t6.*t7.*t10.*t15.*4.0-t10.*t11.*t12.*t13.*t15.*4.0).*(1.0./2.0)-AddedMmgy.*(t37+t38-t6.*t7.*t10.*t15.*8.0-t10.*t11.*t12.*t13.*t15.*8.0).*(1.0./4.0)),dphif.*(AddedMmgy.*t113.*(1.0./2.0)-AddedMmgy.*(t37+t38-t10.*t26.*8.0-t7.*t8.*t32.*8.0).*(1.0./4.0))-dphit.*t123+dthetat.*(AddedMmgy.*t112.*(1.0./2.0)-AddedMmgy.*(t34+t117-t10.*t11.*t13.*t15.*8.0).*(1.0./4.0))-dthetaf.*(AddedMmgy.*t116.*(1.0./2.0)-AddedMmgy.*(t34+t117+t118+t119-t6.*t8.*t11.*t15.*8.0-t10.*t11.*t13.*t15.*8.0).*(1.0./4.0))+dpsif.*(AddedMmgy.*t111.*(1.0./2.0)-AddedMmgy.*(t10.*t20.*-8.0+t7.*t8.*t18.*8.0+t4.*t8.*t11.*t13.*8.0).*(1.0./4.0)),0.0,0.0,0.0,0.0,-dthetat.*t128+dphif.*(AddedMmgz.*t63.*(1.0./2.0)-AddedMmgz.*(t40+t64-t7.*t10.*t12.*t13.*8.0).*(1.0./4.0))-dphit.*(AddedMmgz.*(t39-t7.*t10.*t12.*t13.*4.0).*(1.0./2.0)-AddedMmgz.*(t40-t7.*t10.*t12.*t13.*8.0).*(1.0./4.0))+dthetaf.*(AddedMmgz.*(t41+t42+t50-t6.*t10.*t11.*t12.*4.0).*(1.0./2.0)-AddedMmgz.*(t43+t44+t7.*t10.*t13.*8.0-t6.*t10.*t11.*t12.*8.0).*(1.0./4.0)),dphif.*(AddedMmgz.*t144.*(1.0./2.0)-AddedMmgz.*(t43+t44-t10.*t12.*t13.*8.0+t7.*t8.*t13.*t14.*8.0).*(1.0./4.0))-dphit.*t128-dthetaf.*(AddedMmgz.*t133.*(1.0./2.0)-AddedMmgz.*t153.*(1.0./4.0))-dthetat.*(AddedMmgz.*(t39+t85-t86).*(1.0./2.0)-AddedMmgz.*(t40-t124+t135).*(1.0./4.0)),0.0,0.0,0.0,0.0,dthetaf.*(t77-AddedMmgy.*t53.*t82+AddedMmgx.*t46.*(t27+t48+t49+t73+t75-t10.*t18.*4.0-t7.*t8.*t20.*4.0-t4.*t7.*t8.*t13.*4.0))-dphif.*(AddedMmgx.*t46.*t138+AddedMmgy.*t53.*t140-AddedMmgz.*t61.*(t39+t64+t85-t7.*t10.*t12.*t13.*8.0))-dpsif.*t93+dphit.*(AddedMmgx.*t46.*t56+AddedMmgy.*t53.*t58-AddedMmgz.*t61.*t63)-dthetat.*(-AddedMmgx.*t46.*t108+AddedMmgy.*t53.*t113+AddedMmgz.*t61.*(t41+t42+t68-t10.*t12.*t13.*4.0)),-dpsif.*t171+dthetaf.*t162-dphit.*(-AddedMmgx.*t56.*t111+AddedMmgy.*t58.*t139+AddedMmgz.*t63.*t143)+dthetat.*(AddedMmgx.*t108.*t111+AddedMmgy.*t113.*t139-AddedMmgz.*t143.*t144)+dphif.*(-AddedMmgx.*t111.*t138+AddedMmgy.*t139.*t140+AddedMmgz.*t143.*(t39+t64+t85-t124)),0.0,0.0,0.0,0.0,-dthetaf.*(AddedMmgx.*t46.*t152+AddedMmgy.*t53.*t155+AddedMmgz.*t61.*(-t40+t124+t134+t136+t137-t8.*t13.*t14.*8.0))-dpsif.*(AddedMmgy.*t53.*(t96+t98+t99-t7.*t8.*t18.*4.0-t4.*t8.*t11.*t13.*4.0-t4.*t7.*t10.*t12.*t13.*4.0)-AddedMmgx.*t46.*t102)+dphif.*(t77-AddedMmgy.*t53.*t82+AddedMmgx.*t46.*t147)-dphit.*(AddedMmgx.*t46.*t156+AddedMmgy.*t53.*t157+AddedMmgz.*t61.*t158)+dthetat.*(AddedMmgx.*t46.*t104+AddedMmgy.*t53.*t116+AddedMmgz.*t61.*t133),dphif.*t162+dpsif.*t175-dphit.*(AddedMmgx.*t111.*t156-AddedMmgy.*t139.*t157+AddedMmgz.*t143.*t158)+dthetat.*(AddedMmgx.*t104.*t111-AddedMmgy.*t116.*t139+AddedMmgz.*t133.*t143)-dthetaf.*(AddedMmgx.*t111.*t152-AddedMmgy.*t139.*t155+AddedMmgz.*t143.*t153),0.0,0.0,0.0,0.0,dphit.*(AddedMmgx.*t46.*t53-AddedMmgy.*t46.*t53)-dpsif.*(AddedMmgx.*t46.*t106+AddedMmgy.*t53.*t112)+dthetaf.*(AddedMmgx.*t46.*t102-AddedMmgy.*t53.*t163)-dphif.*t93-dthetat.*(AddedMmgx.*t46.*(t94+t95-t7.*t8.*t26.*4.0)+AddedMmgy.*t53.*t111),dphit.*(AddedMmgx.*t53.*t111+AddedMmgy.*t46.*t139)-dpsif.*(AddedMmgx.*t106.*t111-AddedMmgy.*t112.*t139)-dthetat.*(AddedMmgx.*t111.*t139-AddedMmgy.*t111.*t139)-dphif.*t171+dthetaf.*t175,0.0,0.0,0.0,0.0],[6, 6]);
