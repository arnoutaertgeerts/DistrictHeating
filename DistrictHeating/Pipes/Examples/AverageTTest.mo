within DistrictHeating.Pipes.Examples;
model AverageTTest
  "Unit test for pipe with average temperature calculation - Changing T, same m_flow"
  extends Modelica.Icons.Example;

  AvgTempPlugFlowPipe averageTempPlugFlowPipe(
    pipeLength=40,
    m_flow_nominal=4.5,
    redeclare package Medium = Annex60.Media.Water,
    TInit(displayUnit="degC") = 343.15,
    pipeDiameter=0.05)
    annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  IDEAS.Fluid.Sources.FixedBoundary sink(
    redeclare package Medium = Annex60.Media.Water,
    nPorts=1,
    T=343.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={92,0})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=4.5,
    T_start=343.15)
    annotation (Placement(transformation(extent={{-34,-10},{-14,10}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut(redeclare package Medium =
        Annex60.Media.Water, m_flow_nominal=4.5,
    T_start=343.15)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  IDEAS.Fluid.Sources.FixedBoundary bou(
    redeclare package Medium = Annex60.Media.Water,
    T=343.15,
    nPorts=1)
    annotation (Placement(transformation(extent={{-104,-10},{-84,10}})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_T hea(
    redeclare package Medium = Annex60.Media.Water,
    m_flow_nominal=4.5,
    dp_nominal=0)
    annotation (Placement(transformation(extent={{-54,34},{-34,54}})));
  IDEAS.Fluid.Movers.Pump pump(
    use_onOffSignal=false,
    redeclare package Medium = Buildings.Media.Water,
    m_flow_nominal=4.5,
    m_flow(start=4.5))
    annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));
  Modelica.Blocks.Sources.Pulse pulse(
    amplitude=20,
    width=40,
    offset=273 + 70,
    period=200,
    startTime=200)
    annotation (Placement(transformation(extent={{-92,40},{-72,60}})));
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
  connect(bou.ports[1], pump.port_a) annotation (Line(
      points={{-84,0},{-70,0}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pump.port_b, hea.port_a) annotation (Line(
      points={{-50,0},{-48,0},{-48,24},{-60,24},{-60,44},{-54,44}},
      color={0,127,255},
      smooth=Smooth.None));
  connect(pulse.y, hea.TSet) annotation (Line(
      points={{-71,50},{-56,50}},
      color={0,0,127},
      smooth=Smooth.None));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}), graphics), __Dymola_Commands(file=
          "Pipes/Examples/Simulate and plot.mos" "Simulate and plot"));
end AverageTTest;
