
// Remove the line above if you are submitting to GradeScope for a grade. But leave it if you only want to check
// that your code compiles and the autograder can access your public methods.

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using GameAI;

// All the Fuzz
using Tochas.FuzzyLogic;
using Tochas.FuzzyLogic.MembershipFunctions;
using Tochas.FuzzyLogic.Evaluators;
using Tochas.FuzzyLogic.Mergers;
using Tochas.FuzzyLogic.Defuzzers;
using Tochas.FuzzyLogic.Expressions;

namespace GameAICourse
{

    public class FuzzyVehicle : AIVehicle
    {

        // TODO create some Fuzzy Set enumeration types, and member variables for:
        // Fuzzy Sets (input and output), one or more Fuzzy Value Sets, and Fuzzy
        // Rule Sets for each output.
        // Also, create some methods to instantiate each of the member variables

        // Here are some basic examples to get you started
        enum FzOutputThrottle { Brake, Coast, Accelerate }
        enum FzOutputWheel { TurnLeft, Straight, TurnRight }
        enum FzInputSpeed { Slow, Medium, Fast }
        enum FzInputVehicleOrientation { Left, Straight, Right }

        FuzzySet<FzInputSpeed> fzSpeedSet;
        FuzzySet<FzInputVehicleOrientation> fzVehicleOrientationSet;

        FuzzySet<FzOutputThrottle> fzThrottleSet;
        FuzzyRuleSet<FzOutputThrottle> fzThrottleRuleSet;

        FuzzySet<FzOutputWheel> fzWheelSet;
        FuzzyRuleSet<FzOutputWheel> fzWheelRuleSet;

        FuzzyValueSet fzInputValueSet = new FuzzyValueSet();

        // These are used for debugging (see ApplyFuzzyRules() call
        // in Update()
        FuzzyValueSet mergedThrottle = new FuzzyValueSet();
        FuzzyValueSet mergedWheel = new FuzzyValueSet();

        private FuzzySet<FzInputSpeed> GetSpeedSet()
        {
            FuzzySet<FzInputSpeed> set = new FuzzySet<FzInputSpeed>();

            // TODO: Add some membership functions for each state
            IMembershipFunction slowFx = new ShoulderMembershipFunction(0f, new Coords(0f, 1f), new Coords(30f, 0f), 65f);
            IMembershipFunction mediumFx = new TriangularMembershipFunction(new Coords(30f, 0f), new Coords(45f, 1f), new Coords(65f, 0f));
            IMembershipFunction fastFx = new ShoulderMembershipFunction(0f, new Coords(45f, 0f), new Coords(65f, 1f), 65f);

            set.Set(FzInputSpeed.Slow, slowFx);
            set.Set(FzInputSpeed.Medium, mediumFx);
            set.Set(FzInputSpeed.Fast, fastFx);
            return set;
        }

        private FuzzySet<FzOutputThrottle> GetThrottleSet()
        {

            FuzzySet<FzOutputThrottle> set = new FuzzySet<FzOutputThrottle>();

            // TODO: Add some membership functions for each state
            IMembershipFunction brakeFx = new ShoulderMembershipFunction(-70f, new Coords(-30f, 1f), new Coords(-10f, 0f), 70f);
            IMembershipFunction coastFx = new TriangularMembershipFunction(new Coords(-70f, 0f), new Coords(0f, 1f), new Coords(70f, 0f));
            IMembershipFunction accelerateFx = new ShoulderMembershipFunction(-70f, new Coords(50f, 0f), new Coords(70f, 1f), 70f);

            set.Set(FzOutputThrottle.Accelerate, accelerateFx);
            set.Set(FzOutputThrottle.Brake, brakeFx);
            set.Set(FzOutputThrottle.Coast, coastFx);
            return set;
        }

        private FuzzySet<FzOutputWheel> GetWheelSet()
        {

            FuzzySet<FzOutputWheel> set = new FuzzySet<FzOutputWheel>();

            // TODO: Add some membership functions for each state
            IMembershipFunction turnLeftFx = new ShoulderMembershipFunction(-.7f, new Coords(-.7f, 1f), new Coords(-.3f, 0f), .7f);
            IMembershipFunction straightFx = new TriangularMembershipFunction(new Coords(-.7f, 0f), new Coords(0f, 1f), new Coords(.7f, 0f));
            IMembershipFunction turnRightFx = new ShoulderMembershipFunction(-.7f, new Coords(.3f, 0f), new Coords(.7f, 1f), .7f);

            set.Set(FzOutputWheel.TurnLeft, turnLeftFx);
            set.Set(FzOutputWheel.Straight, straightFx);
            set.Set(FzOutputWheel.TurnRight, turnRightFx);
            return set;
        }

        private FuzzySet<FzInputVehicleOrientation> GetVehicleOrientationSet()
        {
            FuzzySet<FzInputVehicleOrientation> set = new FuzzySet<FzInputVehicleOrientation>();

            // TODO: Add some membership functions for each state
            IMembershipFunction leftFx = new ShoulderMembershipFunction(30f, new Coords(30f, 1f), new Coords(2f, 0f), -30f);
            IMembershipFunction straightFx = new TriangularMembershipFunction(new Coords(30f, 0f), new Coords(0f, 1f), new Coords(-30f, 0f));
            IMembershipFunction rightFx = new ShoulderMembershipFunction(30f, new Coords(-2f, 0f), new Coords(-30f, 1f), -30f);

            set.Set(FzInputVehicleOrientation.Left, leftFx);
            set.Set(FzInputVehicleOrientation.Straight, straightFx);
            set.Set(FzInputVehicleOrientation.Right, rightFx);
            return set;
        }


        private FuzzyRule<FzOutputThrottle>[] GetThrottleRules()
        {

            FuzzyRule<FzOutputThrottle>[] rules =
            {
                // TODO: Add some rules. Here is an example
                // (Note: these aren't necessarily good rules)
                If(And(FzInputSpeed.Slow, FzInputVehicleOrientation.Straight)).Then(FzOutputThrottle.Accelerate),
                If(And(FzInputSpeed.Slow, FzInputVehicleOrientation.Left)).Then(FzOutputThrottle.Accelerate),
                If(And(FzInputSpeed.Slow, FzInputVehicleOrientation.Right)).Then(FzOutputThrottle.Accelerate),
                If(And(FzInputSpeed.Fast, FzInputVehicleOrientation.Right)).Then(FzOutputThrottle.Brake),
                If(And(FzInputSpeed.Fast, FzInputVehicleOrientation.Left)).Then(FzOutputThrottle.Brake),
                If(And(FzInputSpeed.Fast, FzInputVehicleOrientation.Straight)).Then(FzOutputThrottle.Coast),
                If(And(FzInputSpeed.Medium, FzInputVehicleOrientation.Straight)).Then(FzOutputThrottle.Accelerate),
                If(And(FzInputSpeed.Medium, FzInputVehicleOrientation.Left)).Then(FzOutputThrottle.Brake),
                If(And(FzInputSpeed.Medium, FzInputVehicleOrientation.Right)).Then(FzOutputThrottle.Brake),
                // More example syntax
                //If(And(FzInputSpeed.Fast, Not(FzFoo.Bar)).Then(FzOutputThrottle.Accelerate),
            };

            return rules;
        }

        private FuzzyRule<FzOutputWheel>[] GetWheelRules()
        {

            FuzzyRule<FzOutputWheel>[] rules =
            {
                // TODO: Add some rules.
                If(FzInputVehicleOrientation.Left).Then(FzOutputWheel.TurnLeft),
                If(FzInputVehicleOrientation.Straight).Then(FzOutputWheel.Straight),
                If(FzInputVehicleOrientation.Right).Then(FzOutputWheel.TurnRight),
            };

            return rules;
        }

        private FuzzyRuleSet<FzOutputThrottle> GetThrottleRuleSet(FuzzySet<FzOutputThrottle> throttle)
        {
            var rules = this.GetThrottleRules();
            return new FuzzyRuleSet<FzOutputThrottle>(throttle, rules);
        }

        private FuzzyRuleSet<FzOutputWheel> GetWheelRuleSet(FuzzySet<FzOutputWheel> wheel)
        {
            var rules = this.GetWheelRules();
            return new FuzzyRuleSet<FzOutputWheel>(wheel, rules);
        }


        protected override void Awake()
        {
            base.Awake();

            StudentName = "Yochitha Jeeiah Prakash";

            // Only the AI can control. No humans allowed!
            IsPlayer = false;

        }

        protected override void Start()
        {
            base.Start();

            // TODO: You can initialize a bunch of Fuzzy stuff here
            fzSpeedSet = this.GetSpeedSet();
            fzVehicleOrientationSet = this.GetVehicleOrientationSet();

            fzThrottleSet = this.GetThrottleSet();
            fzThrottleRuleSet = this.GetThrottleRuleSet(fzThrottleSet);

            fzWheelSet = this.GetWheelSet();
            fzWheelRuleSet = this.GetWheelRuleSet(fzWheelSet);
        }

        System.Text.StringBuilder strBldr = new System.Text.StringBuilder();

        override protected void Update()
        {

            // TODO Do all your Fuzzy stuff here and then
            // pass your fuzzy rule sets to ApplyFuzzyRules()

            // Remove these once you get your fuzzy rules working.
            // You can leave one hardcoded while you work on the other.
            // Both steering and throttle must be implemented with variable
            // control and not fixed/hardcoded!

            //HardCodeSteering(0f);
            //HardCodeThrottle(1f);

            // Simple example of fuzzification of vehicle state
            // The Speed is fuzzified and stored in fzInputValueSet
            var ac = pathTracker.pathCreator.path.GetPointAtDistance(pathTracker.distanceTravelled + 10f) - transform.position;
            var ab = transform.forward * 5;
            float vehicleDirection = Vector3.SignedAngle(ac, ab, Vector3.up);
            
            fzSpeedSet.Evaluate(Speed, fzInputValueSet);
            fzVehicleOrientationSet.Evaluate(vehicleDirection, fzInputValueSet);

            // ApplyFuzzyRules evaluates your rules and assigns Thottle and Steering accordingly
            // Also, some intermediate values are passed back for debugging purposes
            // Throttle = someValue; //[-1f, 1f] -1 is full brake, 0 is neutral, 1 is full throttle
            // Steering = someValue; // [-1f, 1f] -1 if full left, 0 is neutral, 1 is full right
            ApplyFuzzyRules<FzOutputThrottle, FzOutputWheel>(
                fzThrottleRuleSet,
                fzWheelRuleSet,
                fzInputValueSet,
                // access to intermediate state for debugging
                out var throttleRuleOutput,
                out var wheelRuleOutput,
                ref mergedThrottle,
                ref mergedWheel
                );

            // Use vizText for debugging output
            // You might also use Debug.DrawLine() to draw vectors on Scene view
            /*if (vizText != null)
            {
                strBldr.Clear();

                strBldr.AppendLine($"Demo Output");
                strBldr.AppendLine($"Comment out before submission");

                // You will probably want to selectively enable/disable printing
                // of certain fuzzy states or rules

                AIVehicle.DiagnosticPrintFuzzyValueSet<FzInputSpeed>(fzInputValueSet, strBldr);
                AIVehicle.DiagnosticPrintFuzzyValueSet<FzInputVehicleOrientation>(fzInputValueSet, strBldr);
                AIVehicle.DiagnosticPrintRuleSet<FzOutputThrottle>(fzThrottleRuleSet, throttleRuleOutput, strBldr);
                AIVehicle.DiagnosticPrintRuleSet<FzOutputWheel>(fzWheelRuleSet, wheelRuleOutput, strBldr);
                Debug.DrawLine(transform.position, transform.position + transform.forward * 5, Color.red);
                Debug.DrawLine(transform.position, pathTracker.pathCreator.path.GetPointAtDistance(pathTracker.distanceTravelled + 10f), Color.blue);
                vizText.text = strBldr.ToString();
            }*/

            // recommend you keep the base Update call at the end, after all your FuzzyVehicle code so that
            // control inputs can be processed properly (e.g. Throttle, Steering)
            base.Update();
        }

    }
}
