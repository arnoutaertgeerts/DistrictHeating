within DistrictHeating;
model CentralProduction
  extends Modelica.Icons.Example;

  parameter Modelica.SIunits.Length depth = 1.5 "Buried depth of pipes";

  ArenbergDistrictHeating.Consumer CW(redeclare package Medium =
        Annex60.Media.Water, filePath="M:/Documents/Holdub/MatData/CW.mat")
    annotation (Placement(transformation(extent={{160,60},{180,80}})));
  ArenbergDistrictHeating.Consumer NatNavorsing(redeclare package Medium =
        Annex60.Media.Water, filePath=
        "M:/Documents/Holdub/MatData/NavNatuurkunde.mat")
    annotation (Placement(transformation(extent={{40,-60},{60,-80}})));
  ArenbergDistrictHeating.Consumer NatKandidatuur(redeclare package Medium =
        Annex60.Media.Water, filePath=
        "M:/Documents/Holdub/MatData/KanNatuurkunde.mat")
    annotation (Placement(transformation(extent={{40,60},{60,80}})));
  ArenbergDistrictHeating.Consumer Geo(filePath=
        "M:/Documents/Holdub/MatData/Geo.mat", redeclare package Medium =
        Annex60.Media.Water)
    annotation (Placement(transformation(extent={{-20,60},{0,80}})));
  ArenbergDistrictHeating.Consumer Scheikunde(filePath=
        "M:/Documents/Holdub/MatData/Scheikunde.mat", redeclare package Medium
      = Annex60.Media.Water) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-86,10})));
  IDEAS.Fluid.Sources.FixedBoundary bou(nPorts=2, redeclare package Medium =
        Annex60.Media.Water)                      annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={130,-90})));
  IDEAS.Fluid.Production.IdealHeater idealHeater(m_flow_nominal=65)
    annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=90 + 273.15)
    annotation (Placement(transformation(extent={{76,-68},{96,-48}})));
  DistrictHeating.Interfaces.DHConnection LCentr(
    measureSupplyT=false,
    m_flow_nominal=62.5,
    length=13,
    Di=0.2,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={130,-10})));
  DistrictHeating.Interfaces.DHConnection LOost(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=20.55,
    length=62,
    Di=0.125,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={150,10})));
  DistrictHeating.Interfaces.DHConnection DCW(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    m_flow_nominal=10.26,
    length=68,
    Di=0.1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={170,38})));
  DistrictHeating.Interfaces.DHConnection DAud(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    m_flow_nominal=9.63,
    length=150,
    Di=0.1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={190,10})));
  DistrictHeating.Interfaces.DHConnection LWest1(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    m_flow_nominal=37.6,
    length=80,
    Di=0.150) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={110,10})));
  DistrictHeating.Interfaces.DHConnection LWest2(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    Di=0.150,
    m_flow_nominal=26.8,
    length=57) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={70,10})));
  DistrictHeating.Interfaces.DHConnection DNatNav(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    m_flow_nominal=10.6,
    length=33,
    Di=0.1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={50,-40})));
  DistrictHeating.Interfaces.DHConnection LWest3(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    m_flow_nominal=16.55,
    length=40,
    Di=0.125) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={20,10})));
  DistrictHeating.Interfaces.DHConnection DScheikunde(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    m_flow_nominal=11.33,
    length=164,
    Di=0.1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-38,10})));
  DistrictHeating.Interfaces.DHConnection DGeo(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    m_flow_nominal=5.21,
    length=25,
    Di=0.065) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-10,42})));
  DistrictHeating.Interfaces.DHConnection DKanNat(
    measureSupplyT=false,
    redeclare package Medium = Annex60.Media.Water,
    redeclare DistrictHeating.Pipes.DoublePipes.TwinPipeGround
      districtHeatingPipe,
    includePipes=false,
    m_flow_nominal=10.17,
    length=53,
    Di=0.1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,42})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1Prod(
    redeclare package Medium = Annex60.Media.Water,
    tau=0.001,
    m_flow_nominal=65) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={124,-46})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2Prod(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=65,
    tau=0.001) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={136,-48})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1Schei(
    redeclare package Medium = Annex60.Media.Water,
    tau=0.001,
    m_flow_nominal=11.3) annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=0,
        origin={-62,16})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2Schei(
    redeclare package Medium = Annex60.Media.Water,
    tau=0.001,
    m_flow_nominal=11.3) annotation (Placement(transformation(
        extent={{4,-4},{-4,4}},
        rotation=180,
        origin={-62,4})));
equation
  connect(bou.ports[1], idealHeater.port_a) annotation (Line(
      points={{128,-80},{124,-80},{124,-76},{120,-76}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.TSet, realExpression.y) annotation (Line(
      points={{106,-58},{97,-58}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(LCentr.port_b1, LOost.port_a1) annotation (Line(
      points={{124,0},{124,16},{140,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LCentr.port_a2, LOost.port_b2) annotation (Line(
      points={{136,0},{136,4},{140,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DCW.port_b1, CW.port_a1) annotation (Line(
      points={{164,48},{164,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DCW.port_a2, CW.port_b1) annotation (Line(
      points={{176,48},{176,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LOost.port_b1, DCW.port_a1) annotation (Line(
      points={{160,16},{164,16},{164,28}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LOost.port_a2, DCW.port_b2) annotation (Line(
      points={{160,4},{176,4},{176,28}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LOost.port_b1, DAud.port_a1) annotation (Line(
      points={{160,16},{180,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LOost.port_a2, DAud.port_b2) annotation (Line(
      points={{160,4},{180,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LOost.port_a1, LWest1.port_a1) annotation (Line(
      points={{140,16},{120,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LOost.port_b2, LWest1.port_b2) annotation (Line(
      points={{140,4},{120,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LWest1.port_b1, LWest2.port_a1) annotation (Line(
      points={{100,16},{80,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LWest1.port_a2, LWest2.port_b2) annotation (Line(
      points={{100,4},{80,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DNatNav.port_b1, NatNavorsing.port_a1) annotation (Line(
      points={{44,-50},{44,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DNatNav.port_a2, NatNavorsing.port_b1) annotation (Line(
      points={{56,-50},{56,-60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DNatNav.port_b2, LWest2.port_b2) annotation (Line(
      points={{56,-30},{56,-26},{96,-26},{96,4},{80,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DNatNav.port_a1, LWest2.port_a1) annotation (Line(
      points={{44,-30},{44,-14},{84,-14},{84,16},{80,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LWest2.port_b1, LWest3.port_a1) annotation (Line(
      points={{60,16},{30,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LWest2.port_a2, LWest3.port_b2) annotation (Line(
      points={{60,4},{30,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DScheikunde.port_a1, LWest3.port_b1) annotation (Line(
      points={{-28,16},{10,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DScheikunde.port_b2, LWest3.port_a2) annotation (Line(
      points={{-28,4},{10,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DGeo.port_b1, Geo.port_a1) annotation (Line(
      points={{-16,52},{-16,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DGeo.port_a2, Geo.port_b1) annotation (Line(
      points={{-4,52},{-4,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DGeo.port_a1, LWest3.port_b1) annotation (Line(
      points={{-16,32},{-16,16},{10,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DGeo.port_b2, LWest3.port_a2) annotation (Line(
      points={{-4,32},{-4,4},{10,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DKanNat.port_a1, LWest3.port_a1) annotation (Line(
      points={{44,32},{44,16},{30,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DKanNat.port_b2, LWest3.port_b2) annotation (Line(
      points={{56,32},{56,4},{30,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DKanNat.port_b1, NatKandidatuur.port_a1) annotation (Line(
      points={{44,52},{44,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DKanNat.port_a2, NatKandidatuur.port_b1) annotation (Line(
      points={{56,52},{56,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.port_b, T1Prod.port_a) annotation (Line(
      points={{120,-64},{124,-64},{124,-50}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1Prod.port_b, LCentr.port_a1) annotation (Line(
      points={{124,-42},{124,-20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(LCentr.port_b2, T2Prod.port_a) annotation (Line(
      points={{136,-20},{136,-44}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T2Prod.port_b, bou.ports[2]) annotation (Line(
      points={{136,-52},{136,-80},{132,-80}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DScheikunde.port_b1, T1Schei.port_a) annotation (Line(
      points={{-48,16},{-58,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1Schei.port_b, Scheikunde.port_a1) annotation (Line(
      points={{-66,16},{-76,16}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(DScheikunde.port_a2, T2Schei.port_b) annotation (Line(
      points={{-48,4},{-58,4}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T2Schei.port_a, Scheikunde.port_b1) annotation (Line(
      points={{-66,4},{-76,4}},
      color={0,127,255},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(extent={{-100,-100},{260,100}},
          preserveAspectRatio=false),
        graphics), Icon(coordinateSystem(extent={{-100,-100},{260,100}})),
    experiment(
      StartTime=1,
      StopTime=3.6e+006,
      Interval=100),
    __Dymola_experimentSetupOutput);
end CentralProduction;
