#!/usr/bin/perl

use strict;
use warnings;

use Slic3r::XS;
use Test::More tests => 10;
{
    {
        my $test_string = "{if{3 == 4}} string";

        my $result = Slic3r::ConditionalGCode::apply_math($test_string);
        is $result, "", 'If statement with nested bracket removes on false resolution.';
    }
    {
        my $test_string = "{if{3 == 4}} string\notherstring";

        my $result = Slic3r::ConditionalGCode::apply_math($test_string);
        is $result, "otherstring", 'if false only removes up to newline.';
    }

    {
        my $test_string = "{if{3 == 3}} string";

        my $result = Slic3r::ConditionalGCode::apply_math($test_string);
        is $result, " string", 'If statement with nested bracket removes itself only on resulting true, does not strip text outside of brackets.';
    }

    {
        my $test_string = "{if 3 > 2} string";

        my $result = Slic3r::ConditionalGCode::apply_math($test_string);
        is $result, " string", 'If statement with nested bracket removes itself only on resulting true, does not strip text outside of brackets.';
    }
    {
        my $test_string = "{if{3 == 3}}string";

        my $result = Slic3r::ConditionalGCode::apply_math($test_string);
        is $result, "string", 'If statement with nested bracket removes itself only on resulting true.';
    }
    {
        my $test_string = "M104 S{4*5}; Sets temp to {4*5}";

        my $result = Slic3r::ConditionalGCode::apply_math($test_string);
        is $result, "M104 S20; Sets temp to 20", 'Bracket replacement works with math ops';
    }
    {
        my $test_string = "M104 S\\{a\\}; Sets temp to {4*5}";

        my $result = Slic3r::ConditionalGCode::apply_math($test_string);
        is $result, "M104 S{a}; Sets temp to 20", 'Escaped string emittal.';
    }
    {
        my $test_string = "M104 S{a}; Sets temp to {4*5}";

        my $result = Slic3r::ConditionalGCode::apply_math($test_string);
        is $result, "M104 S{a}; Sets temp to 20", 'string (minus brackets) on failure to parse.';
    }
    {
        my $config = Slic3r::Config->new;
        $config->set('infill_extruder', 2);
        $config->normalize;
        my $test_string = "{if [infill_extruder] == 2}M104 S210";
        my $pp = Slic3r::GCode::PlaceholderParser->new;
        $pp->apply_config($config);
        my $interim = $pp->process($test_string);
        is $interim, "{if 2 == 2}M104 S210", 'Placeholder parser works inside conditional gcode.';

        my $result = Slic3r::ConditionalGCode::apply_math($interim);
        is $result, "M104 S210", 'If statement with nested bracket removes itself only on resulting true, does not strip text outside of brackets.';
    }

}
