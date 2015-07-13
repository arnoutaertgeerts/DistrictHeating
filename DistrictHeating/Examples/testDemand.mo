within DistrictHeating.Examples;
model testDemand

  extends Modelica.Icons.Example;

  IDEAS.Fluid.Sources.FixedBoundary bou(nPorts=2, redeclare package Medium =
        Annex60.Media.Water,
    T=70 + 273.15)                                annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-34,-44})));
  IDEAS.Fluid.Production.IdealHeater idealHeater(m_flow_nominal=65)
    annotation (Placement(transformation(extent={{-64,-34},{-44,-14}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=90 + 273.15)
    annotation (Placement(transformation(extent={{-88,-22},{-68,-2}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2Prod(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=65,
    tau=0.001) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={-14,10})));
  DistrictHeating.Interfaces.DHConnection dHConnection(
    L=100,
    redeclare package Medium = Annex60.Media.Water,
    Di=0.01,
    m_flow_nominal=65,
    redeclare DistrictHeating.Pipes.DoublePipes.DHDeltaCircuit
      districtHeatingPipe(redeclare
        DistrictHeating.Pipes.DoublePipes.Configurations.TwinPipeGround
        baseConfiguration, nSeg=1),
    tau=1)                          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-20,32})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_T hea(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=65,
    dp_nominal=10)
    annotation (Placement(transformation(extent={{-44,60},{-24,80}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=70 + 273.15)
    annotation (Placement(transformation(extent={{-84,66},{-64,86}})));
  IDEAS.Fluid.Movers.Pump pump(useInput=true, m_flow_nominal=65) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-40,-2})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=0.01)
    annotation (Placement(transformation(extent={{-90,8},{-70,28}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TDelta(
    redeclare package Medium = Annex60.Media.Water,
    tau=0.001,
    m_flow_nominal=65) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-54,64})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1Prod(
    redeclare package Medium = Annex60.Media.Water,
    tau=0.001,
    m_flow_nominal=65) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={-40,16})));
  IDEAS.Fluid.Sources.FixedBoundary bou1(
                                        nPorts=2, redeclare package Medium =
        Annex60.Media.Water,
    T=70 + 273.15)                                annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={68,-46})));
  IDEAS.Fluid.Production.IdealHeater idealHeater1(
                                                 m_flow_nominal=65)
    annotation (Placement(transformation(extent={{38,-36},{58,-16}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(
                                                        y=90 + 273.15)
    annotation (Placement(transformation(extent={{14,-24},{34,-4}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T2Prod1(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=65,
    tau=0.001) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=270,
        origin={88,8})));
  DistrictHeating.Interfaces.DHConnection dHConnection1(
    L=100,
    redeclare package Medium = Annex60.Media.Water,
    Di=0.01,
    m_flow_nominal=65,
    redeclare DistrictHeating.Pipes.DoublePipes.DHWallenten districtHeatingPipe(
        redeclare
        DistrictHeating.Pipes.DoublePipes.Configurations.TwinPipeGround
        baseConfiguration, nSeg=1),
    tau=1)                          annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={82,30})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_T hea1(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=65,
    dp_nominal=10)
    annotation (Placement(transformation(extent={{58,58},{78,78}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=70 + 273.15)
    annotation (Placement(transformation(extent={{18,64},{38,84}})));
  IDEAS.Fluid.Movers.Pump pump1(useInput=true, m_flow_nominal=65) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={62,-4})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TWallenten(
    redeclare package Medium = Annex60.Media.Water,
    tau=0.001,
    m_flow_nominal=65) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={48,62})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T1Prod3(
    redeclare package Medium = Annex60.Media.Water,
    tau=0.001,
    m_flow_nominal=65) annotation (Placement(transformation(
        extent={{-4,-4},{4,4}},
        rotation=90,
        origin={62,14})));
equation
  connect(bou.ports[1],idealHeater. port_a) annotation (Line(
      points={{-36,-34},{-40,-34},{-40,-30},{-44,-30}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.TSet,realExpression. y) annotation (Line(
      points={{-58,-12},{-67,-12}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(T2Prod.port_b, bou.ports[2]) annotation (Line(
      points={{-14,6},{-14,-22},{-32,-22},{-32,-34}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(dHConnection.port_b2, T2Prod.port_a) annotation (Line(
      points={{-14,22},{-14,14}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hea.port_b, dHConnection.port_a2) annotation (Line(
      points={{-24,70},{-14,70},{-14,42}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hea.TSet, realExpression1.y) annotation (Line(
      points={{-46,76},{-63,76}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(pump.m_flowSet, realExpression2.y) annotation (Line(
      points={{-50.4,-2},{-60,-2},{-60,18},{-69,18}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(dHConnection.port_b1, TDelta.port_a) annotation (Line(
      points={{-26,42},{-26,50},{-54,50},{-54,60}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TDelta.port_b, hea.port_a) annotation (Line(
      points={{-54,68},{-54,70},{-44,70}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater.port_b, pump.port_a) annotation (Line(
      points={{-44,-18},{-42,-18},{-42,-12},{-40,-12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump.port_b, T1Prod.port_a) annotation (Line(
      points={{-40,8},{-40,12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1Prod.port_b, dHConnection.port_a1) annotation (Line(
      points={{-40,20},{-40,22},{-26,22}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(bou1.ports[1], idealHeater1.port_a) annotation (Line(
      points={{66,-36},{62,-36},{62,-32},{58,-32}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater1.TSet, realExpression3.y) annotation (Line(
      points={{44,-14},{35,-14}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(T2Prod1.port_b, bou1.ports[2]) annotation (Line(
      points={{88,4},{88,-24},{70,-24},{70,-36}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(dHConnection1.port_b2, T2Prod1.port_a) annotation (Line(
      points={{88,20},{88,12}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hea1.port_b, dHConnection1.port_a2) annotation (Line(
      points={{78,68},{88,68},{88,40}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(hea1.TSet, realExpression4.y) annotation (Line(
      points={{56,74},{39,74}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(dHConnection1.port_b1, TWallenten.port_a) annotation (Line(
      points={{76,40},{76,48},{48,48},{48,58}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TWallenten.port_b, hea1.port_a) annotation (Line(
      points={{48,66},{48,68},{58,68}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(idealHeater1.port_b, pump1.port_a) annotation (Line(
      points={{58,-20},{60,-20},{60,-14},{62,-14}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump1.port_b, T1Prod3.port_a) annotation (Line(
      points={{62,6},{62,10}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(T1Prod3.port_b, dHConnection1.port_a1) annotation (Line(
      points={{62,18},{62,20},{76,20}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(realExpression2.y, pump1.m_flowSet) annotation (Line(
      points={{-69,18},{-12,18},{-12,16},{46,16},{46,-4},{51.6,-4}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics));
end testDemand;
