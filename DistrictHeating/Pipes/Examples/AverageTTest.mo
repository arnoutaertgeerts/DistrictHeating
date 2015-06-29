within DistrictHeating.Pipes.Examples;
model AverageTTest "Unit test for pipe with average temperature calculation"
  extends Modelica.Icons.Example;

  AverageTempPlugFlowPipe averageTempPlugFlowPipe(
    pipeLength=40,
    pipeDiameter=0.2,
    m_flow_nominal=4.5,
    redeclare package Medium = Annex60.Media.Water,
    TInit(displayUnit="degC") = 343.15)
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  IDEAS.Fluid.Sources.FixedBoundary sink(
    redeclare package Medium = Annex60.Media.Water,
    nPorts=1,
    T=343.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={92,0})));
  Modelica.Blocks.Sources.Step step(
    height=20,
    startTime=50,
    offset=70 + 273)
    annotation (Placement(transformation(extent={{-98,72},{-78,92}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=4.5)
    annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=4.5)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  IDEAS.Fluid.Sources.FixedBoundary bou(
    redeclare package Medium = Annex60.Media.Water,
    nPorts=1,
    T=343.15)
    annotation (Placement(transformation(extent={{-104,-10},{-84,10}})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_T hea(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=4.5,
    dp_nominal=0)
    annotation (Placement(transformation(extent={{-54,34},{-34,54}})));
  IDEAS.Fluid.Movers.FlowMachine_m_flow fan(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=4.5)
    annotation (Placement(transformation(extent={{-76,-10},{-56,10}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=4.5)
    annotation (Placement(transformation(extent={{-102,26},{-82,46}})));
equation
  connect(averageTempPlugFlowPipe.port_a, TIn.port_b) annotation (Line(
      points={{0,0},{-14,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(averageTempPlugFlowPipe.port_b, TOut.port_a) annotation (Line(
      points={{20,0},{40,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(sink.ports[1], TOut.port_b) annotation (Line(
      points={{82,1.33227e-015},{54,1.33227e-015},{54,0},{60,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(TIn.port_a, hea.port_b) annotation (Line(
      points={{-34,0},{-34,44}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(step.y, hea.TSet) annotation (Line(
      points={{-77,82},{-68,82},{-68,50},{-56,50}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(bou.ports[1], fan.port_a) annotation (Line(
      points={{-84,0},{-76,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(fan.port_b, hea.port_a) annotation (Line(
      points={{-56,0},{-50,0},{-50,28},{-62,28},{-62,44},{-54,44}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(realExpression.y, fan.m_flow_in) annotation (Line(
      points={{-81,36},{-66.2,36},{-66.2,12}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), __Dymola_Commands(file=
          "Pipes/Examples/Simulate and plot.mos" "Simulate and plot"));
end AverageTTest;
