within DistrictHeating.Pipes.DoublePipes;
package Examples
  model TwinPipes
    //Extensions
    extends Modelica.Icons.Example;

    TwinPipeGround pipe1(
      redeclare package Medium = Annex60.Media.Water,
      L=100,
      Pipe1(lambdaIns=0.026),
      Pipe2(lambdaIns=0.026),
      nSeg=150) annotation (Placement(transformation(extent={{-10,0},{10,28}})));
    IDEAS.Fluid.Sources.FixedBoundary bou2(
      T=273.15 + 70,
      redeclare package Medium = IDEAS.Media.Water.Simple,
      use_T=false,
      nPorts=1) annotation (Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=90,
          origin={-80,-8})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort T1In(m_flow_nominal=0.1, redeclare
        package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-28,42})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort T2In(m_flow_nominal=0.1, redeclare
        package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={44,-14})));
    Annex60.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
          Annex60.Media.Water, m_flow_nominal=0.5)
      annotation (Placement(transformation(extent={{-62,32},{-42,52}})));
    Modelica.Blocks.Sources.Pulse pulse(
      width=5,
      period=86400,
      offset=273.15 + 50,
      amplitude=20)
      annotation (Placement(transformation(extent={{-82,62},{-90,70}})));
    Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater(
      dp_nominal=0,
      redeclare package Medium = Annex60.Media.Water,
      m_flow_nominal=0.1)
      annotation (Placement(transformation(extent={{-88,32},{-68,52}})));
    Annex60.Fluid.HeatExchangers.HeaterCooler_T    idealHeater1(
      dp_nominal=0,
      redeclare package Medium = Annex60.Media.Water,
      m_flow_nominal=0.1)
      annotation (Placement(transformation(extent={{62,10},{82,30}})));
    Modelica.Blocks.Sources.Pulse pulse1(
      period=86400,
      startTime=7200,
      amplitude=-0.09,
      width=60,
      offset=0.1)
      annotation (Placement(transformation(extent={{-74,62},{-66,70}})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort T1Out(
      m_flow_nominal=0.1,
      redeclare package Medium = IDEAS.Media.Water.Simple,
      tau=0) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={32,20})));
    IDEAS.Fluid.Sensors.TemperatureTwoPort T2Out(m_flow_nominal=0.1, redeclare
        package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={-26,8})));
    Modelica.Blocks.Sources.Constant const(k=273.15 + 40)
      annotation (Placement(transformation(extent={{78,36},{70,44}})));
    Modelica.Blocks.Sources.Constant const1(k=273.15 + 10) annotation (
        Placement(transformation(
          extent={{-4,-4},{4,4}},
          rotation=90,
          origin={0,-14})));
  equation
    connect(fan.port_b, T1In.port_a) annotation (Line(
        points={{-42,42},{-38,42}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(idealHeater.port_b,fan. port_a) annotation (Line(
        points={{-68,42},{-62,42}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(T1In.port_b, pipe1.port_a1) annotation (Line(
        points={{-18,42},{-14,42},{-14,20},{-10,20}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pulse.y, idealHeater.TSet) annotation (Line(
        points={{-90.4,66},{-96,66},{-96,48},{-90,48}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(pipe1.port_b2, T2Out.port_a) annotation (Line(
        points={{-10,8},{-16,8}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(const.y, idealHeater1.TSet) annotation (Line(
        points={{69.6,40},{54.4,40},{54.4,26},{60,26}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(const1.y, pipe1.Tg) annotation (Line(
        points={{2.22045e-016,-9.6},{0,-9.6},{0,-0.2}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(T1Out.port_b, idealHeater1.port_a) annotation (Line(
        points={{42,20},{62,20}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(idealHeater1.port_b, T2In.port_a) annotation (Line(
        points={{82,20},{88,20},{88,-14},{54,-14}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(T2Out.port_b, idealHeater.port_a) annotation (Line(
        points={{-36,8},{-90,8},{-90,42},{-88,42}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pulse1.y, fan.m_flow_in) annotation (Line(
        points={{-65.6,66},{-52.2,66},{-52.2,54}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(bou2.ports[1], idealHeater.port_a) annotation (Line(
        points={{-80,-4},{-80,8},{-90,8},{-90,42},{-88,42}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipe1.port_b1, T1Out.port_a) annotation (Line(
        points={{10,20},{22,20}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipe1.port_a2, T2In.port_b) annotation (Line(
        points={{10,8},{20,8},{20,-14},{34,-14}},
        color={0,127,255},
        smooth=Smooth.None));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics));
  end TwinPipes;
end Examples;