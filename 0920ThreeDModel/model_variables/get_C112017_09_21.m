function out1 = get_C112017_09_21(in1,in2)
%GET_C112017_09_21
%    OUT1 = GET_C112017_09_21(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    21-Sep-2017 10:02:25

dphif = in2(4,:);
dphig = in2(9,:);
dphit = in2(7,:);
dpsif = in2(6,:);
dpsig = in2(11,:);
dthetaa = in2(12,:);
dthetaf = in2(5,:);
dthetag = in2(10,:);
dthetat = in2(8,:);
phif = in1(4,:);
phig = in1(9,:);
phit = in1(7,:);
psif = in1(6,:);
psig = in1(11,:);
thetaa = in1(12,:);
thetaf = in1(5,:);
thetag = in1(10,:);
thetat = in1(8,:);
t2 = thetaf-thetat;
t3 = pi.*(1.0./2.0);
t4 = t3+thetag-thetat;
t5 = psif-psig;
t6 = phif-phig;
t7 = sin(t2);
t8 = thetaa-thetag;
t9 = cos(t8);
t10 = cos(t6);
t11 = cos(t5);
t12 = sin(t6);
t13 = sin(t5);
t14 = cos(t4);
t15 = cos(phif);
t16 = sin(psif);
t17 = cos(psif);
t18 = sin(phif);
t19 = -t3+thetaf;
t20 = cos(t19);
t21 = t16.*t18;
t22 = t15.*t17.*t20;
t23 = t21+t22;
t24 = phif-phit;
t25 = cos(t2);
t26 = sin(t24);
t27 = cos(t24);
t28 = sin(t4);
t29 = t15.*t16;
t63 = t17.*t18.*t20;
t30 = t29-t63;
t31 = t10.*t25.*t27;
t32 = t10.*t26;
t33 = t12.*t25.*t27;
t34 = t10.*t27;
t40 = t12.*t26;
t50 = t12.*t25.*t26;
t35 = t31+t34-t40-t50;
t36 = t7.*t27.*t28;
t37 = t12.*t27;
t38 = t10.*t25.*t26;
t39 = t32+t33+t37+t38;
t41 = t32+t33;
t42 = t31-t40;
t43 = t15.*t17;
t44 = t16.*t18.*t20;
t45 = t43+t44;
t46 = sin(t8);
t47 = t14.*t39;
t48 = t7.*t26.*t28;
t49 = t47+t48;
t51 = t17.*t18;
t55 = t15.*t16.*t20;
t52 = t51-t55;
t53 = t25.*t28;
t56 = t7.*t10.*t14;
t54 = t53-t56;
t57 = t13.*t54;
t58 = t7.*t11.*t12;
t59 = t57+t58;
t60 = t7.*t10.*t11;
t61 = t7.*t12.*t13.*t14;
t62 = t60+t61;
t64 = t9.*t30.*t62.*(2.2e1./1.25e2);
t65 = t13.*t49;
t66 = t11.*t35;
t67 = t65+t66;
t68 = t9.*t23.*t67.*(2.2e1./1.25e2);
t69 = t14.*t42;
t70 = t36+t69;
t71 = t11.*t41;
t72 = sin(t19);
t73 = t25.*t27.*4.0;
t74 = t14.*t35;
t75 = t36+t74;
t76 = t11.*t39;
t181 = t13.*t75;
t182 = t76-t181;
t77 = t9.*t17.*t72.*t182.*(2.2e1./1.25e2);
t78 = t7.*t26.*4.0;
t79 = t7.*t10.*t26;
t80 = t7.*t12.*t27;
t81 = t79+t80;
t82 = t14.*t25.*t26;
t83 = t7.*t10.*t27;
t87 = t7.*t12.*t26;
t84 = t83-t87;
t85 = t25.*t27.*t28;
t86 = t7.*t27.*4.0;
t88 = t14.*t25.*t27;
t89 = t7.*t10.*t27.*t28;
t90 = t28.*t35;
t101 = t7.*t14.*t27;
t91 = t90-t101;
t92 = t46.*t91.*(1.1e1./1.25e2);
t93 = t11.*t75;
t94 = t13.*t39;
t95 = t93+t94;
t129 = t9.*t95.*(1.1e1./1.25e2);
t96 = t73+t92-t129;
t97 = t11.*t70;
t98 = t13.*t41;
t99 = t97+t98;
t100 = t28.*t42;
t102 = t13.*t35;
t126 = t11.*t49;
t127 = t102-t126;
t103 = t9.*t127.*(1.1e1./1.25e2);
t104 = t25.*t26.*4.0;
t105 = t28.*t39;
t128 = t7.*t14.*t26;
t106 = t105-t128;
t107 = t46.*t106.*(1.1e1./1.25e2);
t108 = t103+t104+t107;
t109 = t7.*t10.*t13;
t124 = t7.*t11.*t12.*t14;
t110 = t109-t124;
t111 = t9.*t110.*(1.1e1./1.25e2);
t112 = t7.*t12.*t28.*t46.*(1.1e1./1.25e2);
t113 = t111+t112;
t114 = t7.*4.0;
t115 = t14.*t25;
t116 = t7.*t10.*t28;
t117 = t115+t116;
t118 = t46.*t117.*(1.1e1./1.25e2);
t119 = t11.*t54;
t123 = t7.*t12.*t13;
t120 = t119-t123;
t121 = t9.*t120.*(1.1e1./1.25e2);
t122 = t114+t118+t121;
t125 = t100-t101;
t130 = t37+t38;
t131 = t10.*t27.*2.0;
t132 = t10.*t25.*t27.*2.0;
t146 = t12.*t26.*2.0;
t147 = t12.*t25.*t26.*2.0;
t133 = t131+t132-t146-t147;
t134 = t7.*t10.*t11.*t14;
t135 = t123+t134;
t136 = t9.*t135.*(1.1e1./1.25e2);
t234 = t7.*t10.*t28.*t46.*(1.1e1./1.25e2);
t137 = t136-t234;
t138 = t30.*t137.*2.0;
t139 = t9.*(t102-t126).*(1.1e1./1.25e2);
t140 = t104+t107+t139;
t141 = t10.*t26.*2.0;
t142 = t12.*t27.*2.0;
t143 = t10.*t25.*t26.*2.0;
t144 = t12.*t25.*t27.*2.0;
t145 = t141+t142+t143+t144;
t148 = t17.*t25.*t26.*t72.*6.2e2;
t149 = t46.*t125.*(1.1e1./1.25e2);
t179 = t9.*t99.*(1.1e1./1.25e2);
t150 = t73+t149-t179;
t151 = t46.*t110.*(1.1e1./1.25e2);
t152 = t9.*t117.*(1.1e1./1.25e2);
t153 = t46.*t99.*(1.1e1./1.25e2);
t154 = t9.*t125.*(1.1e1./1.25e2);
t155 = t9.*t106.*(1.1e1./1.25e2);
t156 = t46.*t95.*(1.1e1./1.25e2);
t157 = t9.*t91.*(1.1e1./1.25e2);
t158 = t7.*t12.*t13.*t27;
t159 = t10.*t13.*t25;
t160 = t7.*t14;
t161 = t7.*t28;
t162 = t10.*t14.*t25;
t163 = t12.*t13.*t25;
t164 = t14.*t81;
t165 = t13.*t84;
t166 = t28.*t81;
t167 = t7.*t10.*t14.*t26;
t168 = t7.*t12.*t14.*t27;
t169 = t7.*t12.*t13.*t26;
t170 = t7.*t10.*t26.*t28;
t171 = t7.*t12.*t27.*t28;
t172 = t12.*t25.*t28.*t46.*(1.1e1./1.25e2);
t173 = t7.*t12.*t14.*t46.*(1.1e1./1.25e2);
t174 = t7.*t23.*t26.*6.2e2;
t175 = t13.*t81;
t176 = t28.*t84;
t177 = t7.*t12.*t14.*t26;
t178 = t7.*t10.*t13.*t26;
t184 = t13.*t70;
t180 = t71-t184;
t183 = t9.*t52.*t59.*(2.2e1./1.25e2);
t185 = t9.*t16.*t72.*t182.*(2.2e1./1.25e2);
t186 = t36+t69-t88-t89;
t187 = t46.*t186.*(1.1e1./1.25e2);
t225 = t7.*t10.*t14.*t27;
t188 = t85-t100+t101-t225;
t348 = t11.*t188;
t189 = t158-t348;
t190 = t9.*t189.*(1.1e1./1.25e2);
t191 = -t86+t187+t190;
t192 = t12.*t25.*t28;
t193 = t7.*t12.*t14;
t194 = t192+t193;
t195 = t46.*t194.*(1.1e1./1.25e2);
t196 = t7.*t12.*t28;
t349 = t12.*t14.*t25;
t197 = t196-t349;
t198 = t11.*t197;
t199 = t9.*(t159+t198).*(1.1e1./1.25e2);
t200 = t195+t199;
t286 = t10.*t25.*t28;
t201 = t53-t56+t160-t286;
t202 = t46.*t201.*(1.1e1./1.25e2);
t203 = t115+t116-t161-t162;
t350 = t11.*t203;
t204 = t163-t350;
t205 = t9.*t204.*(1.1e1./1.25e2);
t285 = t25.*4.0;
t206 = t202+t205-t285;
t212 = t25.*t26.*t28;
t207 = t105-t128+t164-t212;
t351 = t11.*t207;
t208 = t165-t351;
t209 = t9.*t208.*(1.1e1./1.25e2);
t210 = t47+t48-t82-t166;
t352 = t46.*t210.*(1.1e1./1.25e2);
t211 = t78+t209-t352;
t213 = -t47-t48+t82+t170+t171;
t214 = t46.*t213.*(1.1e1./1.25e2);
t215 = t7.*t11.*t12.*t28;
t282 = t11.*t12.*t14.*t25;
t216 = t159+t215-t282;
t217 = t9.*t216.*(1.1e1./1.25e2);
t218 = t172+t173+t217;
t296 = t14.*t84;
t219 = t85-t90+t101-t296;
t339 = t11.*t219;
t220 = t175-t339;
t221 = t9.*t220.*(1.1e1./1.25e2);
t222 = t36+t74-t88-t176;
t223 = t46.*t222.*(1.1e1./1.25e2);
t224 = -t86+t221+t223;
t226 = t7.*t12.*t26.*t28;
t227 = t36+t74-t88-t89+t226;
t228 = t46.*t227.*(1.1e1./1.25e2);
t276 = t46.*t120.*(1.1e1./1.25e2);
t229 = t152-t276;
t273 = t7.*t9.*t12.*t28.*(1.1e1./1.25e2);
t230 = t151-t273;
t231 = t153+t154;
t232 = t46.*t127.*(1.1e1./1.25e2);
t233 = t156+t157;
t235 = t11.*t14.*t35;
t236 = t94+t235;
t237 = t9.*t236.*(1.1e1./1.25e2);
t360 = t28.*t35.*t46.*(1.1e1./1.25e2);
t238 = t237-t360;
t239 = t13.*t42;
t361 = t11.*t14.*t41;
t240 = t239-t361;
t241 = t9.*t240.*(1.1e1./1.25e2);
t242 = t28.*t41.*t46.*(1.1e1./1.25e2);
t243 = t241+t242;
t359 = t11.*t14.*t39;
t244 = t102-t359;
t245 = t9.*t244.*(1.1e1./1.25e2);
t246 = t28.*t39.*t46.*(1.1e1./1.25e2);
t247 = t245+t246;
t248 = t14.*t130;
t249 = t48+t248;
t250 = t11.*t249;
t251 = t34-t50;
t302 = t13.*t251;
t252 = t250-t302;
t253 = t9.*t252.*(1.1e1./1.25e2);
t303 = t28.*t130;
t254 = t128-t303;
t255 = t46.*t254.*(1.1e1./1.25e2);
t256 = -t104+t253+t255;
t308 = t28.*t133;
t257 = t101-t308;
t258 = t46.*t257.*(1.1e1./1.25e2);
t259 = t14.*t133;
t260 = t36+t259;
t261 = t11.*t260;
t262 = t13.*t145;
t263 = t261+t262;
t264 = t9.*t263.*(1.1e1./1.25e2);
t265 = -t73+t258+t264;
t266 = t45.*t137.*2.0;
t267 = t14.*t145;
t268 = t48+t267;
t269 = t13.*t133;
t304 = t11.*t268;
t270 = t9.*(t269-t304).*(1.1e1./1.25e2);
t305 = t28.*t145;
t271 = t128-t305;
t306 = t46.*t271.*(1.1e1./1.25e2);
t272 = t104+t270-t306;
t274 = t7.*t9.*t11.*t12.*t28.*(1.1e1./1.25e2);
t275 = t46.*t54.*(1.1e1./1.25e2);
t313 = t46.*t70.*(1.1e1./1.25e2);
t314 = t9.*t11.*t125.*(1.1e1./1.25e2);
t277 = t153+t154-t313-t314;
t278 = t46.*t49.*(1.1e1./1.25e2);
t279 = t9.*t11.*t106.*(1.1e1./1.25e2);
t280 = -t155+t232+t278+t279;
t309 = t46.*t75.*(1.1e1./1.25e2);
t310 = t9.*t11.*t91.*(1.1e1./1.25e2);
t281 = t156+t157-t309-t310;
t283 = t82+t170+t171;
t284 = t46.*t283.*(1.1e1./1.25e2);
t287 = t161+t162;
t288 = t11.*t287;
t289 = t163+t288;
t290 = t9.*t289.*(1.1e1./1.25e2);
t291 = t88+t89;
t292 = t46.*t291.*(1.1e1./1.25e2);
t293 = t82+t166;
t294 = t46.*t293.*(1.1e1./1.25e2);
t295 = t7.*t26.*t52.*6.2e2;
t297 = t88+t176;
t298 = t46.*t297.*(1.1e1./1.25e2);
t299 = t7.*t16.*t27.*t72.*6.2e2;
t300 = t46.*(t100-t101).*(1.1e1./1.25e2);
t301 = t73-t179+t300;
t307 = t20.*t25.*t26.*6.2e2;
t311 = t151+t173-t273+t274;
t412 = t9.*t11.*t117.*(1.1e1./1.25e2);
t312 = t152+t275-t276-t412;
t315 = t85-t296;
t369 = t11.*t315;
t316 = t175-t369;
t370 = t9.*t316.*(1.1e1./1.25e2);
t317 = t86+t298-t370;
t318 = t85+t177-t225;
t371 = t11.*t318;
t319 = t158+t178-t371;
t320 = t88+t89-t226;
t321 = t46.*t320.*(1.1e1./1.25e2);
t372 = t9.*t319.*(1.1e1./1.25e2);
t322 = t86+t321-t372;
t323 = t85-t225;
t421 = t11.*t323;
t324 = t158-t421;
t422 = t9.*t324.*(1.1e1./1.25e2);
t325 = t86+t292-t422;
t326 = t164-t212;
t368 = t11.*t326;
t327 = t9.*(t165-t368).*(1.1e1./1.25e2);
t328 = t78+t294+t327;
t329 = t159-t282;
t330 = t9.*t329.*(1.1e1./1.25e2);
t331 = t172+t330;
t332 = t167+t168-t212;
t333 = t11.*t332;
t346 = t7.*t10.*t13.*t27;
t334 = t169+t333-t346;
t367 = t9.*t334.*(1.1e1./1.25e2);
t335 = t78+t284-t367;
t336 = t160-t286;
t337 = t46.*t336.*(1.1e1./1.25e2);
t338 = -t285+t290+t337;
t340 = t85-t90+t101+t177-t225;
t366 = t11.*t340;
t341 = t158+t178-t366;
t342 = t9.*t341.*(1.1e1./1.25e2);
t343 = -t86+t228+t342;
t344 = t105-t128+t167+t168-t212;
t345 = t11.*t344;
t347 = t7.*t20.*t27.*6.2e2;
t353 = t7.*t18.*t27.*t72.*6.2e2;
t354 = t7.*t15.*t26.*t72.*6.2e2;
t355 = t9.*t15.*t59.*t72.*(2.2e1./1.25e2);
t356 = t9.*t18.*t62.*t72.*(2.2e1./1.25e2);
t357 = t9.*t18.*t72.*t180.*(2.2e1./1.25e2);
t358 = t155-t232;
t362 = t23.*t211;
t363 = t169+t345-t346;
t374 = t9.*t363.*(1.1e1./1.25e2);
t364 = t78+t214-t374;
t365 = t17.*t72.*t224;
t373 = t52.*t211;
t375 = t15.*t72.*t364;
t376 = t18.*t72.*t218;
t377 = t18.*t72.*t200;
t378 = t20.*t317;
t379 = t20.*t322;
t380 = t15.*t72.*t328;
t381 = t15.*t72.*t335;
t382 = t45.*t113;
t383 = t52.*t140;
t384 = t52.*t122;
t385 = t16.*t72.*t96;
t396 = t45.*t301;
t386 = t382+t383+t384+t385-t396;
t387 = t30.*t113;
t388 = t23.*t140;
t389 = t23.*t122;
t390 = t20.*t96;
t391 = t18.*t72.*t301;
t392 = t15.*t72.*t140;
t393 = t15.*t72.*t122;
t401 = t18.*t72.*t113;
t394 = t390+t391+t392+t393-t401;
t397 = t30.*t301;
t398 = t17.*t72.*t96;
t395 = t387+t388+t389-t397-t398;
t399 = t9.*t52.*t59.*(1.1e1./1.25e2);
t400 = t9.*t16.*t72.*t182.*(1.1e1./1.25e2);
t402 = t9.*t15.*t59.*t72.*(1.1e1./1.25e2);
t403 = t9.*t18.*t62.*t72.*(1.1e1./1.25e2);
t404 = t9.*t18.*t72.*t180.*(1.1e1./1.25e2);
t405 = t9.*t20.*t182.*(1.1e1./1.25e2);
t406 = t402+t403+t404+t405-t9.*t15.*t67.*t72.*(1.1e1./1.25e2);
t407 = t394.*t406.*2.0;
t408 = t9.*t30.*t62.*(1.1e1./1.25e2);
t409 = t9.*t23.*t67.*(1.1e1./1.25e2);
t410 = t9.*t30.*t180.*(1.1e1./1.25e2);
t411 = t9.*t17.*t72.*t182.*(1.1e1./1.25e2);
t413 = t7.*t52.*2.0;
t414 = t25.*t26.*t52.*2.0;
t415 = t16.*t25.*t27.*t72.*2.0;
t450 = t25.*t27.*t45.*2.0;
t416 = t413+t414+t415-t450;
t417 = t7.*t52.*4.0;
t418 = t25.*t26.*t52.*4.0;
t419 = t16.*t25.*t27.*t72.*4.0;
t453 = t25.*t27.*t45.*4.0;
t420 = t417+t418+t419-t453;
t423 = t52.*t328;
t424 = t16.*t72.*t317;
t425 = t7.*t15.*t72.*2.0;
t426 = t20.*t25.*t27.*2.0;
t427 = t18.*t25.*t27.*t72.*2.0;
t428 = t15.*t25.*t26.*t72.*2.0;
t429 = t425+t426+t427+t428;
t430 = t7.*t15.*t72.*4.0;
t431 = t20.*t25.*t27.*4.0;
t432 = t18.*t25.*t27.*t72.*4.0;
t433 = t15.*t25.*t26.*t72.*4.0;
t434 = t430+t431+t432+t433;
t435 = t7.*t23.*2.0;
t436 = t23.*t25.*t26.*2.0;
t442 = t25.*t27.*t30.*2.0;
t443 = t17.*t25.*t27.*t72.*2.0;
t437 = t435+t436-t442-t443;
t438 = t7.*t23.*4.0;
t439 = t23.*t25.*t26.*4.0;
t447 = t25.*t27.*t30.*4.0;
t448 = t17.*t25.*t27.*t72.*4.0;
t440 = t438+t439-t447-t448;
t441 = t23.*t328;
t444 = t23.*t25.*t27.*4.0;
t445 = t25.*t26.*t30.*4.0;
t446 = t17.*t25.*t26.*t72.*2.0;
t449 = t17.*t25.*t26.*t72.*4.0;
t451 = t25.*t27.*t52.*4.0;
t452 = t25.*t26.*t45.*4.0;
t454 = t15.*t72.*t113.*2.0;
t455 = t23.*t113.*2.0;
t456 = t20.*t25.*t26.*2.0;
t457 = t18.*t25.*t26.*t72.*4.0;
t458 = t20.*t25.*t26.*4.0;
t459 = t52.*t113.*2.0;
t460 = t7.*t23.*t26.*2.0;
t461 = t7.*t23.*t26.*4.0;
t462 = t20.*t224;
t463 = t25.*t52.*2.0;
t464 = t7.*t27.*t45.*2.0;
t465 = t25.*t52.*4.0;
t466 = t7.*t27.*t45.*4.0;
t467 = t15.*t25.*t72.*2.0;
t468 = t15.*t25.*t72.*4.0;
t469 = t45.*t137;
out1 = reshape([0.0,0.0,0.0,dthetaf.*(t441-t23.*t335-t17.*t72.*t317+t17.*t72.*t322)-dthetat.*(t362+t365+t30.*t200-t30.*t218-t23.*t364-t17.*t72.*t343),0.0,0.0,0.0,-dthetaf.*(t423+t424-t52.*t335-t16.*t72.*t322)+dthetat.*(t373+t45.*t200-t45.*t218-t52.*t364-t16.*t72.*t224+t16.*t72.*t343),0.0,0.0,0.0,-dthetat.*(t375-t376+t377+t462-t20.*t343-t15.*t72.*t211)-dthetaf.*(t378-t379+t380-t381),-dphif.*(-t138+t148+t7.*t30.*6.2e2-t23.*t113.*4.0+t30.*t122.*2.0+t30.*t140.*4.0+t23.*t150.*2.0-t23.*t265.*2.0+t23.*t25.*t27.*1.24e3+t25.*t26.*t30.*1.24e3+t17.*t72.*t272.*2.0)-dthetag.*(t23.*(t152+t275-t46.*t120.*(1.1e1./1.25e2)-t9.*t11.*t117.*(1.1e1./1.25e2)).*-2.0+t23.*t280.*2.0+t30.*t277.*2.0+t30.*(t151+t173+t274-t7.*t9.*t12.*t28.*(1.1e1./1.25e2)).*2.0+t17.*t72.*t281.*2.0)-dphig.*(t138+t455+t23.*t238.*2.0-t30.*t243.*2.0-t17.*t72.*t247.*2.0)+dphit.*(t148+t23.*t96.*2.0-t30.*t256.*2.0+t23.*t25.*t27.*6.2e2+t25.*t26.*t30.*6.2e2+t17.*t72.*t140.*2.0)+dthetaf.*(t174+t23.*(t78+t294+t9.*(t165-t11.*(t164-t25.*t26.*t28)).*(1.1e1./1.25e2))-t23.*t25.*6.2e2-t30.*(t86+t292+t9.*(t11.*(t85-t7.*t10.*t14.*t27)-t7.*t12.*t13.*t27).*(1.1e1./1.25e2)).*2.0-t30.*(t172+t9.*(t159-t11.*t12.*t14.*t25).*(1.1e1./1.25e2)).*2.0+t23.*(t78+t284-t9.*(t169+t11.*(t167+t168-t25.*t26.*t28)-t7.*t10.*t13.*t27).*(1.1e1./1.25e2))+t23.*(t25.*-4.0+t290+t46.*(t160-t10.*t25.*t28).*(1.1e1./1.25e2)).*2.0-t17.*t72.*(t86+t298-t9.*(t175-t11.*(t85-t14.*t84)).*(1.1e1./1.25e2))-t7.*t27.*t30.*6.2e2+t17.*t20.*t96.*2.0-t17.*t72.*(t86+t46.*(t88+t89-t7.*t12.*t26.*t28).*(1.1e1./1.25e2)-t9.*(t158+t178-t11.*(t85+t177-t7.*t10.*t14.*t27)).*(1.1e1./1.25e2))+t17.*t20.*t25.*t27.*6.2e2+t7.*t15.*t17.*t72.*6.2e2-t7.*t17.*t27.*t72.*6.2e2+t15.*t17.*t72.*t108.*2.0-t17.*t18.*t72.*t113.*2.0+t15.*t17.*t72.*t122.*2.0+t17.*t18.*t72.*t150.*2.0+t15.*t17.*t25.*t26.*t72.*6.2e2+t17.*t18.*t25.*t27.*t72.*6.2e2)-dthetat.*(t174+t362+t365+t23.*(t78+t214-t9.*(t169+t11.*(t105-t128+t167+t168-t25.*t26.*t28)-t7.*t10.*t13.*t27).*(1.1e1./1.25e2))-t23.*t25.*6.2e2+t30.*t191.*2.0+t23.*t206.*2.0-t30.*t200-t30.*t218+t17.*t72.*(-t86+t228+t9.*(t158+t178-t11.*(t85-t90+t101+t177-t7.*t10.*t14.*t27)).*(1.1e1./1.25e2))-t7.*t27.*t30.*6.2e2-t7.*t17.*t27.*t72.*6.2e2)+dthetaa.*(t23.*t229.*-2.0+t30.*t230.*2.0+t30.*t231.*2.0-t23.*(t155-t46.*t127.*(1.1e1./1.25e2)).*2.0+t17.*t72.*t233.*2.0)-dpsif.*(t64+t68+t77+t7.*t52.*6.2e2+t45.*t113.*2.0+t52.*t108.*2.0+t52.*t122.*2.0-t45.*(t73-t9.*t99.*(1.1e1./1.25e2)+t46.*(t100-t7.*t14.*t27).*(1.1e1./1.25e2)).*2.0-t9.*t23.*t59.*(2.2e1./1.25e2)-t25.*t27.*t45.*6.2e2+t25.*t26.*t52.*6.2e2+t16.*t72.*t96.*2.0+t9.*t30.*t180.*(2.2e1./1.25e2)+t16.*t25.*t27.*t72.*6.2e2)+dpsig.*(t64+t68+t77+t9.*t30.*(t71-t13.*(t36+t14.*(t31-t12.*t26))).*(2.2e1./1.25e2)-t9.*t23.*t59.*(2.2e1./1.25e2)),-dpsif.*(t183+t185+t7.*t23.*6.2e2+t30.*t113.*2.0+t23.*t122.*2.0+t23.*t140.*2.0-t30.*t150.*2.0+t23.*t25.*t26.*6.2e2-t25.*t27.*t30.*6.2e2-t9.*t45.*t62.*(2.2e1./1.25e2)-t9.*t52.*t67.*(2.2e1./1.25e2)-t17.*t72.*t96.*2.0-t9.*t45.*t180.*(2.2e1./1.25e2)-t17.*t25.*t27.*t72.*6.2e2)+dphif.*(-t266+t7.*t45.*6.2e2-t52.*t113.*4.0+t45.*t122.*2.0+t45.*t140.*4.0-t52.*t265.*2.0+t52.*t301.*2.0+t25.*t26.*t45.*1.24e3+t25.*t27.*t52.*1.24e3-t16.*t72.*t272.*2.0-t16.*t25.*t26.*t72.*6.2e2)-dphit.*(t52.*t96.*2.0-t45.*t256.*2.0+t25.*t26.*t45.*6.2e2+t25.*t27.*t52.*6.2e2-t16.*t72.*t140.*2.0-t16.*t25.*t26.*t72.*6.2e2)+dphig.*(t266+t459-t45.*t243.*2.0+t52.*t238.*2.0+t16.*t72.*t247.*2.0)+dthetat.*(t295+t299+t373-t25.*t52.*6.2e2+t45.*t191.*2.0-t45.*t200+t52.*t206.*2.0-t45.*t218+t52.*(t78+t214-t9.*(t169+t345-t7.*t10.*t13.*t27).*(1.1e1./1.25e2))-t7.*t27.*t45.*6.2e2-t16.*t72.*t224-t16.*t72.*t343)+dthetaf.*(-t295-t299+t25.*t52.*6.2e2+t45.*t325.*2.0+t45.*t331.*2.0-t52.*t328-t52.*t335-t52.*t338.*2.0+t7.*t27.*t45.*6.2e2+t16.*t20.*t96.*2.0-t16.*t72.*t317-t16.*t72.*t322+t16.*t20.*t25.*t27.*6.2e2+t7.*t15.*t16.*t72.*6.2e2-t16.*t18.*t72.*t113.*2.0+t15.*t16.*t72.*t122.*2.0+t15.*t16.*t72.*t140.*2.0+t16.*t18.*t72.*t301.*2.0+t15.*t16.*t25.*t26.*t72.*6.2e2+t16.*t18.*t25.*t27.*t72.*6.2e2)-dpsig.*(-t183-t185+t9.*t45.*t62.*(2.2e1./1.25e2)+t9.*t52.*t67.*(2.2e1./1.25e2)+t9.*t45.*t180.*(2.2e1./1.25e2))+dthetaa.*(t45.*t230.*-2.0-t45.*t231.*2.0+t52.*t229.*2.0+t52.*t358.*2.0+t16.*t72.*t233.*2.0)+dthetag.*(t45.*t277.*2.0+t52.*t280.*2.0+t45.*t311.*2.0-t52.*t312.*2.0-t16.*t72.*t281.*2.0),dthetaa.*(t20.*t233.*2.0+t18.*t72.*(t153+t154).*2.0+t15.*t72.*t229.*2.0+t18.*t72.*t230.*2.0+t15.*t72.*t358.*2.0)-dthetaf.*(t347+t353+t354+t378+t379+t380+t381+t72.*t96.*2.0-t7.*t15.*t20.*6.2e2-t15.*t25.*t72.*6.2e2+t25.*t27.*t72.*6.2e2+t18.*t20.*t113.*2.0-t15.*t20.*t122.*2.0-t15.*t20.*t140.*2.0-t18.*t20.*t301.*2.0+t18.*t72.*t325.*2.0+t18.*t72.*t331.*2.0+t15.*t72.*t338.*2.0-t15.*t20.*t25.*t26.*6.2e2-t18.*t20.*t25.*t27.*6.2e2)-dthetag.*(t20.*t281.*2.0-t15.*t72.*t280.*2.0+t18.*t72.*t277.*2.0+t15.*t72.*t312.*2.0+t18.*t72.*t311.*2.0)-dphif.*(t307+t20.*t272.*2.0+t7.*t18.*t72.*6.2e2+t15.*t72.*t113.*4.0+t18.*t72.*t122.*2.0-t18.*t72.*t137.*2.0+t18.*t72.*t140.*4.0+t15.*t72.*t265.*2.0-t15.*t72.*t301.*2.0-t15.*t25.*t27.*t72.*1.24e3+t18.*t25.*t26.*t72.*1.24e3)-dpsif.*(t355+t356+t357+t9.*t20.*t182.*(2.2e1./1.25e2)-t9.*t15.*t67.*t72.*(2.2e1./1.25e2))+dpsig.*(t355+t356+t357+t9.*t20.*(t76-t181).*(2.2e1./1.25e2)-t9.*t15.*t67.*t72.*(2.2e1./1.25e2))+dthetat.*(t347+t353+t354+t375+t376+t377-t20.*t224-t20.*t343+t15.*t72.*(t78+t209-t352)+t15.*t72.*(t202+t205-t285).*2.0-t15.*t25.*t72.*6.2e2-t18.*t72.*t191.*2.0)+dphit.*(t307+t20.*t140.*2.0-t15.*t72.*t96.*2.0-t18.*t72.*t256.*2.0-t15.*t25.*t27.*t72.*6.2e2+t18.*t25.*t26.*t72.*6.2e2)+dphig.*(t454+t20.*t247.*2.0+t18.*t72.*(t241+t242).*2.0-t18.*t72.*t137.*2.0+t15.*t72.*t238.*2.0),dthetaf.*(t386.*(-t423-t424+t45.*t325+t45.*t331-t52.*t338+t16.*t20.*t96-t16.*t18.*t72.*t113+t15.*t16.*t72.*t122+t15.*t16.*t72.*t140+t16.*t18.*t72.*t301).*2.0-t437.*(t460-t23.*t25.*2.0-t7.*t27.*t30.*2.0+t17.*t20.*t25.*t27.*2.0+t7.*t15.*t17.*t72.*2.0-t7.*t17.*t27.*t72.*2.0+t15.*t17.*t25.*t26.*t72.*2.0+t17.*t18.*t25.*t27.*t72.*2.0).*1.0e1-t440.*(t461-t23.*t25.*4.0-t7.*t27.*t30.*4.0+t17.*t20.*t25.*t27.*4.0+t7.*t15.*t17.*t72.*4.0-t7.*t17.*t27.*t72.*4.0+t15.*t17.*t25.*t26.*t72.*4.0+t17.*t18.*t25.*t27.*t72.*4.0).*1.5e2-t394.*(t378+t380+t72.*t96+t18.*t20.*t113-t15.*t20.*t122-t15.*t20.*t140-t18.*t20.*t301+t18.*t72.*t325+t18.*t72.*t331+t15.*t72.*t338).*2.0+t429.*(t467+t7.*t15.*t20.*2.0-t7.*t20.*t27.*2.0-t25.*t27.*t72.*2.0+t15.*t20.*t25.*t26.*2.0+t18.*t20.*t25.*t27.*2.0-t7.*t15.*t26.*t72.*2.0-t7.*t18.*t27.*t72.*2.0).*1.0e1+t434.*(t468+t7.*t15.*t20.*4.0-t7.*t20.*t27.*4.0-t25.*t27.*t72.*4.0+t15.*t20.*t25.*t26.*4.0+t18.*t20.*t25.*t27.*4.0-t7.*t15.*t26.*t72.*4.0-t7.*t18.*t27.*t72.*4.0).*1.5e2+t416.*(t463+t464-t7.*t26.*t52.*2.0+t16.*t20.*t25.*t27.*2.0+t7.*t15.*t16.*t72.*2.0-t7.*t16.*t27.*t72.*2.0+t15.*t16.*t25.*t26.*t72.*2.0+t16.*t18.*t25.*t27.*t72.*2.0).*1.0e1+t420.*(t465+t466-t7.*t26.*t52.*4.0+t16.*t20.*t25.*t27.*4.0+t7.*t15.*t16.*t72.*4.0-t7.*t16.*t27.*t72.*4.0+t15.*t16.*t25.*t26.*t72.*4.0+t16.*t18.*t25.*t27.*t72.*4.0).*1.5e2-t395.*(t441-t30.*t325+t23.*t338-t30.*t331+t17.*t20.*t96-t17.*t72.*t317-t17.*t18.*t72.*t113+t15.*t17.*t72.*t122+t15.*t17.*t72.*t140+t17.*t18.*t72.*t301).*2.0)-dphit.*(t429.*(t456-t15.*t25.*t27.*t72.*2.0+t18.*t25.*t26.*t72.*2.0).*-1.0e1+t437.*(t446+t23.*t25.*t27.*2.0+t25.*t26.*t30.*2.0).*1.0e1+t420.*(t451+t452-t16.*t25.*t26.*t72.*4.0).*1.5e2-t434.*(t457+t458-t15.*t25.*t27.*t72.*4.0).*1.5e2+t395.*(t23.*t96-t30.*t256+t17.*t72.*t140).*2.0-t386.*(-t52.*t96+t45.*t256+t16.*t72.*t140).*2.0+t440.*(t444+t445+t449).*1.5e2+t416.*(t25.*t26.*t45.*2.0+t25.*t27.*t52.*2.0-t16.*t25.*t26.*t72.*2.0).*1.0e1+t394.*(-t20.*t140+t15.*t72.*t96+t18.*t72.*t256).*2.0)-dpsif.*(t407+t386.*(t387+t388+t389-t397-t398+t399+t400-t9.*t45.*t62.*(1.1e1./1.25e2)-t9.*t52.*t67.*(1.1e1./1.25e2)-t9.*t45.*t180.*(1.1e1./1.25e2)).*2.0-t395.*(t382+t383+t384+t385-t396+t408+t409+t410+t411-t9.*t23.*t59.*(1.1e1./1.25e2)).*2.0)+dthetaa.*(t395.*(-t23.*t229+t30.*t230+t30.*t231-t23.*t358+t17.*t72.*t233).*-2.0+t386.*(-t45.*t230-t45.*t231+t52.*t229+t52.*t358+t16.*t72.*t233).*2.0+t394.*(t20.*t233+t18.*t72.*(t153+t154)+t15.*t72.*t229+t18.*t72.*t230+t15.*t72.*t358).*2.0)+dthetag.*(t395.*(t23.*t280+t30.*t277-t23.*t312+t30.*t311+t17.*t72.*t281).*2.0+t386.*(t45.*t277+t52.*t280+t45.*t311-t52.*t312-t16.*t72.*t281).*2.0-t394.*(t20.*t281+t18.*t72.*(t153+t154-t46.*(t36+t69).*(1.1e1./1.25e2)-t9.*t11.*(t100-t101).*(1.1e1./1.25e2))-t15.*t72.*t280+t15.*t72.*t312+t18.*t72.*t311).*2.0)+dphif.*(t429.*(t456+t457+t7.*t18.*t72.*2.0-t15.*t25.*t27.*t72.*4.0).*-1.0e1+t420.*(t7.*t45.*4.0+t25.*t26.*t45.*8.0+t25.*t27.*t52.*8.0-t16.*t25.*t26.*t72.*4.0).*1.5e2+t395.*(-t455+t30.*t122-t30.*t137+t30.*t140.*2.0-t23.*t265+t23.*t301+t17.*t72.*t272).*2.0+t440.*(t449+t7.*t30.*4.0+t23.*t25.*t27.*8.0+t25.*t26.*t30.*8.0).*1.5e2+t416.*(t451+t452+t7.*t45.*2.0-t16.*t25.*t26.*t72.*2.0).*1.0e1-t394.*(t454+t20.*t272+t18.*t72.*t122-t18.*t72.*t137+t18.*t72.*t140.*2.0+t15.*t72.*t265-t15.*t72.*t301).*2.0-t434.*(t458+t7.*t18.*t72.*4.0-t15.*t25.*t27.*t72.*8.0+t18.*t25.*t26.*t72.*8.0).*1.5e2+t437.*(t444+t445+t446+t7.*t30.*2.0).*1.0e1-t386.*(t459+t469-t45.*t122-t45.*t140.*2.0+t52.*t265-t52.*t301+t16.*t72.*t272).*2.0)+dphig.*(t395.*(t23.*t113+t30.*t137+t23.*t238-t30.*t243-t17.*t72.*t247).*2.0+t386.*(t469+t52.*t113-t45.*t243+t52.*t238+t16.*t72.*t247).*2.0+t394.*(t20.*t247+t18.*t72.*(t241+t242)+t15.*t72.*t113-t18.*t72.*t137+t15.*t72.*t238).*2.0)+dthetat.*(t416.*(t463+t464-t7.*t26.*t52.*2.0-t7.*t16.*t27.*t72.*2.0).*-1.0e1-t420.*(t465+t466-t7.*t26.*t52.*4.0-t7.*t16.*t27.*t72.*4.0).*1.5e2+t429.*(-t467+t7.*t20.*t27.*2.0+t7.*t15.*t26.*t72.*2.0+t7.*t18.*t27.*t72.*2.0).*1.0e1+t434.*(-t468+t7.*t20.*t27.*4.0+t7.*t15.*t26.*t72.*4.0+t7.*t18.*t27.*t72.*4.0).*1.5e2-t437.*(-t460+t23.*t25.*2.0+t7.*t27.*t30.*2.0+t7.*t17.*t27.*t72.*2.0).*1.0e1-t440.*(-t461+t23.*t25.*4.0+t7.*t27.*t30.*4.0+t7.*t17.*t27.*t72.*4.0).*1.5e2+t386.*(t373+t45.*t191+t52.*t206-t45.*t218-t16.*t72.*t224).*2.0+t394.*(t376-t462-t18.*t72.*t191+t15.*t72.*t206+t15.*t72.*t211).*2.0+t395.*(t362+t365+t30.*t191+t23.*t206-t30.*t218).*2.0)-dpsig.*(-t407+t386.*(-t399-t400+t9.*t45.*t62.*(1.1e1./1.25e2)+t9.*t52.*t67.*(1.1e1./1.25e2)+t9.*t45.*t180.*(1.1e1./1.25e2)).*2.0+t395.*(t408+t409+t410+t411-t9.*t23.*t59.*(1.1e1./1.25e2)).*2.0)],[4, 4]);