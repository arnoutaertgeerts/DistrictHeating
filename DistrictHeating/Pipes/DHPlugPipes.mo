within DistrictHeating.Pipes;
package DHPlugPipes
  model TwinPipeGround "Twin pipe model for symmetric pipes in the ground"

    //Extensions
    extends BaseClasses.DHPipePlugFlow(
      hs=1/hsInvers,
      ha=1/haInvers);

    //Parameters
    parameter Modelica.SIunits.Length Dc=2.75*Di
      "Outer diameter of the larger circumscribing pipe";
  protected
    parameter Modelica.SIunits.Length rc=Dc/2
      "Outer radius of the larger circumscribing pipe";

  protected
    final parameter Real hsInvers=
      2*lambdaI/lambdaG*Modelica.Math.log(2*Heff/rc) +
      Modelica.Math.log(rc^2/(2*D*ri)) +
      sigma*Modelica.Math.log(rc^4/(rc^4-D^4));
    final parameter Real haInvers=
      Modelica.Math.log(2*D/ri) +
      sigma*Modelica.Math.log((rc^2+D^2)/(rc^2-D^2));
    final parameter Real sigma = (lambdaI-lambdaG)/(lambdaI+lambdaG);
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},
              {100,140}}),
                     graphics={
          Ellipse(
            extent={{100,-100},{-100,100}},
            lineColor={135,135,135},
            fillColor={255,255,255},
            fillPattern=FillPattern.Forward),
          Polygon(
            points={{34,24},{64,14},{34,2},{34,24}},
            smooth=Smooth.None,
            fillColor={255,0,0},
            fillPattern=FillPattern.HorizontalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{30,-90},{-30,-30}},
            lineColor={0,0,255},
            fillColor={0,0,255},
            fillPattern=FillPattern.Sphere),
          Polygon(
            points={{34,20},{56,14},{34,6},{34,20}},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.HorizontalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Line(
            points={{-58,14},{58,14}},
            color={255,0,0},
            smooth=Smooth.None),
          Polygon(
            points={{-32,0},{-62,-10},{-32,-22},{-32,0}},
            smooth=Smooth.None,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Polygon(
            points={{-32,-4},{-54,-10},{-32,-18},{-32,-4}},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.HorizontalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Line(
            points={{60,-10},{-52,-10}},
            color={0,0,255},
            smooth=Smooth.None),
          Ellipse(
            extent={{30,28},{-30,88}},
            lineColor={255,0,0},
            fillColor={255,0,0},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{-100,-140},{100,-114}},
            lineColor={127,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Backward),
          Line(
            points={{-100,-114},{-100,-140},{-100,-114},{100,-114},{100,-140}},
            color={255,255,255},
            smooth=Smooth.None)}));
  end TwinPipeGround;

  model TwinPipeAir "Twin pipe model for symmetric pipes above the ground"

    //Extensions
    extends BaseClasses.DHPipePlugFlow(
      hs=1/hsInvers,
      ha=1/haInvers);

    //Parameters
    parameter Modelica.SIunits.Length Dc
      "Outer diameter of the larger circumscribing pipe";
  protected
    parameter Modelica.SIunits.Length rc=Dc/2
      "Outer radius of the larger circumscribing pipe";

  protected
    parameter Real hsInvers=
      Modelica.Math.log(rc^2/(2*D*ri)) -
      Modelica.Math.log(rc^4/(rc^4-D^4));
    parameter Real haInvers=
      Modelica.Math.log(2*D/ri) -
      Modelica.Math.log((rc^2+D^2)/(rc^2-D^2));

    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},
              {100,140}}),
                     graphics={
          Ellipse(
            extent={{100,-100},{-100,100}},
            lineColor={135,135,135},
            fillColor={255,255,255},
            fillPattern=FillPattern.Forward),
          Polygon(
            points={{34,24},{64,14},{34,2},{34,24}},
            smooth=Smooth.None,
            fillColor={255,0,0},
            fillPattern=FillPattern.HorizontalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Ellipse(
            extent={{30,-90},{-30,-30}},
            lineColor={0,0,255},
            fillColor={0,0,255},
            fillPattern=FillPattern.Sphere),
          Polygon(
            points={{34,20},{56,14},{34,6},{34,20}},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.HorizontalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Line(
            points={{-58,14},{58,14}},
            color={255,0,0},
            smooth=Smooth.None),
          Polygon(
            points={{-32,0},{-62,-10},{-32,-22},{-32,0}},
            smooth=Smooth.None,
            fillColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Polygon(
            points={{-32,-4},{-54,-10},{-32,-18},{-32,-4}},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.HorizontalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Line(
            points={{60,-10},{-52,-10}},
            color={0,0,255},
            smooth=Smooth.None),
          Ellipse(
            extent={{30,28},{-30,88}},
            lineColor={255,0,0},
            fillColor={255,0,0},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{-100,-140},{100,-114}},
            lineColor={85,170,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Backward),
          Line(
            points={{-100,-114},{-100,-140},{-100,-114},{100,-114},{100,-140}},
            color={255,255,255},
            smooth=Smooth.None)}));
  end TwinPipeAir;

  model SoloPipes
    "A symmetrical preinsulated pipe model where each pipe has its own insulation"

    //Extensions
    extends BaseClasses.DHPipePlugFlow(
      hs=1/hsInvers,
      ha=1/haInvers,
      Do=0.25);

    //Parameters
  protected
    parameter Real hsInvers=
      Modelica.Math.log(2*Heff/ro) + beta +
      Modelica.Math.log(sqrt(1 + (Heff/D)^2));
    parameter Real haInvers=
      Modelica.Math.log(2*Heff/ro) + beta -
      Modelica.Math.log(sqrt(1 + (Heff/D)^2));

    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -120},{100,120}}), graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-140},{100,140}}), graphics={
          Ellipse(
            extent={{-40,-20},{40,-100}},
            lineColor={135,135,135},
            fillPattern=FillPattern.Forward,
            fillColor={255,255,255},
            startAngle=0,
            endAngle=360),
          Ellipse(
            extent={{-40,100},{40,20}},
            lineColor={135,135,135},
            fillPattern=FillPattern.Forward,
            fillColor={255,255,255},
            startAngle=0,
            endAngle=360),
          Ellipse(
            extent={{30,30},{-30,90}},
            lineColor={255,0,0},
            fillColor={255,0,0},
            fillPattern=FillPattern.Sphere),
          Ellipse(
            extent={{30,-90},{-30,-30}},
            lineColor={0,0,255},
            fillColor={0,0,255},
            fillPattern=FillPattern.Sphere),
          Rectangle(
            extent={{-100,-140},{100,-114}},
            lineColor={127,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Backward),
          Line(
            points={{-100,-114},{-100,-140},{-100,-114},{100,-114},{100,-140}},
            color={255,255,255},
            smooth=Smooth.None)}));

  end SoloPipes;

  package Examples
    extends Modelica.Icons.ExamplesPackage;

    model TwinPipes
      //Extensions
      extends Modelica.Icons.Example;

      TwinPipeGround twinPipeGround(redeclare package Medium =
            Annex60.Media.Water, L=100)
        annotation (Placement(transformation(extent={{-10,0},{10,28}})));
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
            origin={28,-16})));
      Annex60.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium
          = Annex60.Media.Water, m_flow_nominal=0.5)
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
        annotation (Placement(transformation(extent={{86,-26},{66,-6}})));
      Modelica.Blocks.Sources.Pulse pulse1(
        period=86400,
        offset=0.1,
        startTime=7200,
        amplitude=-0.09,
        width=60)
        annotation (Placement(transformation(extent={{-74,62},{-66,70}})));
      IDEAS.Fluid.Sensors.TemperatureTwoPort T1Out(
        m_flow_nominal=0.1,
        redeclare package Medium = IDEAS.Media.Water.Simple,
        tau=0) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={26,20})));
      IDEAS.Fluid.Sensors.TemperatureTwoPort T2Out(m_flow_nominal=0.1, redeclare
          package Medium = IDEAS.Media.Water.Simple) annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            rotation=0,
            origin={-26,8})));
      Modelica.Blocks.Sources.Constant const(k=273.15 + 40)
        annotation (Placement(transformation(extent={{72,0},{80,8}})));
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
      connect(T1In.port_b, twinPipeGround.port_a1) annotation (Line(
          points={{-18,42},{-14,42},{-14,20},{-10,20}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pulse.y, idealHeater.TSet) annotation (Line(
          points={{-90.4,66},{-96,66},{-96,48},{-90,48}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(T2In.port_b, twinPipeGround.port_a2) annotation (Line(
          points={{18,-16},{14,-16},{14,8},{10,8}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(twinPipeGround.port_b1, T1Out.port_a) annotation (Line(
          points={{10,20},{16,20}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(twinPipeGround.port_b2, T2Out.port_a) annotation (Line(
          points={{-10,8},{-16,8}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(const.y, idealHeater1.TSet) annotation (Line(
          points={{80.4,4},{94,4},{94,-10},{88,-10}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(const1.y, twinPipeGround.Tg) annotation (Line(
          points={{2.22045e-016,-9.6},{0,-9.6},{0,-0.2}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(T1Out.port_b, idealHeater1.port_a) annotation (Line(
          points={{36,20},{96,20},{96,-16},{86,-16}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(idealHeater1.port_b, T2In.port_a) annotation (Line(
          points={{66,-16},{38,-16}},
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
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics));
    end TwinPipes;
  end Examples;
end DHPlugPipes;
