#version 150
//#extension GL_ARB_shading_language_420pack : require
#extension GL_ARB_explicit_attrib_location : require

#define TASK 10
#define ENABLE_OPACITY_CORRECTION 0
#define ENABLE_LIGHTNING 0
#define ENABLE_SHADOWING 0
#define ENABLE_FRONTBACK 1

in vec3 ray_entry_position;

layout(location = 0) out vec4 FragColor;

uniform mat4 Modelview;

uniform sampler3D volume_texture;
uniform sampler2D transfer_texture;

uniform vec3    camera_location;
uniform float   sampling_distance;
uniform float   sampling_distance_ref;
uniform float   iso_value;
uniform vec3    max_bounds;
uniform ivec3   volume_dimensions;

uniform vec3    light_position;
uniform vec3    light_ambient_color;
uniform vec3    light_diffuse_color;
uniform vec3    light_specular_color;
uniform float   light_ref_coef;


bool
inside_volume_bounds(const in vec3 sampling_position)
{
    return (   all(greaterThanEqual(sampling_position, vec3(0.0)))
            && all(lessThanEqual(sampling_position, max_bounds)));
}


float
get_sample_data(vec3 in_sampling_pos)
{
    vec3 obj_to_tex = vec3(1.0) / max_bounds;
    return texture(volume_texture, in_sampling_pos * obj_to_tex).r;
}

float
get_sample_data(float x, float y, float z)
{
    vec3 obj_to_tex = vec3(1.0) / max_bounds;
    return texture(volume_texture, vec3(x,y,z) * obj_to_tex).r;
}


vec3
binary_search(vec3 sampling_pos_low, vec3 sampling_pos_high, float epsilon_threshold, int condition)
{
	vec3 low_border = sampling_pos_low;
	vec3 high_border = sampling_pos_high;
	vec3 new_coordinates = (low_border + high_border)/2;
	float s = get_sample_data(new_coordinates);
	float difference = s - iso_value;
	int i = 0;

	// iterative
	while (difference != 0) {
		if (difference < epsilon_threshold || i >= condition) {
			break;
		}
		else if (difference > 0) {
			high_border = new_coordinates;
		}
		else if (difference < 0) {
			low_border = new_coordinates;
		}
		new_coordinates = (low_border + high_border)/2;
		s = get_sample_data(new_coordinates);
		difference = s - iso_value;
		i++;
	}

	return new_coordinates;
}

vec3
get_gradient(vec3 sampling_pos)
{
    vec3 steps = max_bounds / volume_dimensions;

    float x = (get_sample_data(sampling_pos.x + steps.x, sampling_pos.y, sampling_pos.z) - 
                get_sample_data(sampling_pos.x - steps.x, sampling_pos.y, sampling_pos.z)) / 2;
    float y = (get_sample_data(sampling_pos.x, sampling_pos.y + steps.y, sampling_pos.z) -
                get_sample_data(sampling_pos.x, sampling_pos.y - steps.y, sampling_pos.z)) / 2;
    float z = (get_sample_data(sampling_pos.x, sampling_pos.y, sampling_pos.z + steps.z) -
                get_sample_data(sampling_pos.x, sampling_pos.y, sampling_pos.z - steps.z)) / 2;
    return normalize(vec3(x,y,z));
}

vec3
think_positiv(vec3 sampling_pos)
{
    return vec3(abs(sampling_pos.x),abs(sampling_pos.y),abs(sampling_pos.z));
}

vec4
light_it_up(vec3 sampling_pos, int factor)
{
    float s = get_sample_data(sampling_pos);
    vec3 n = get_gradient(sampling_pos);
    vec3 l = normalize(light_position - sampling_pos);
    vec3 v = normalize(camera_location - sampling_pos);
    vec3 r = normalize(reflect(l, n));
    vec4 color = texture(transfer_texture, vec2(s, s));
    return vec4((light_ambient_color +
        light_diffuse_color * max(dot(l,n), 0.0) + 
        light_specular_color * pow(max(dot(r,v),0.0), light_ref_coef)) * color.xyz * factor, 1);
}

vec4
shadow_it(vec3 sampling_pos, vec4 color, bool compositing)
{
    vec3 l = normalize(light_position - sampling_pos);
    vec3 shadow_steps = -l * sampling_distance;
    vec3 shadow_test_pos = sampling_pos;
            
    bool shadow_inside_volume = true;
    bool shadow_hit = false;

    while (shadow_inside_volume) {
        float shadow_s = get_sample_data(shadow_test_pos);
        float shadow_diff = shadow_s - iso_value;

        if (shadow_diff >= 0) {
            if (shadow_hit) {
                return vec4(0,0,0,1); 
            }
            shadow_hit = true;
        }
        shadow_test_pos += shadow_steps;
        shadow_inside_volume = inside_volume_bounds(shadow_test_pos);
    }
    return color;
}


void main()
{
    /// One step trough the volume
    vec3 ray_increment      = normalize(ray_entry_position - camera_location) * sampling_distance;
    /// Position in Volume
    vec3 sampling_pos       = ray_entry_position + ray_increment; // test, increment just to be sure we are in the volume

    /// Init color of fragment
    vec4 dst = vec4(0.0, 0.0, 0.0, 0.0);

    /// check if we are inside volume
    bool inside_volume = inside_volume_bounds(sampling_pos);
    
    if (!inside_volume)
        discard;

#if TASK == 10
    vec4 max_val = vec4(0.0, 0.0, 0.0, 0.0);
    
    while (inside_volume) 
    {      
        // get sample
        float s = get_sample_data(sampling_pos);
                
        // apply the transfer functions to retrieve color and opacity
        vec4 color = texture(transfer_texture, vec2(s, s));
           
        // this is the example for maximum intensity projection
        max_val.r = max(color.r, max_val.r);
        max_val.g = max(color.g, max_val.g);
        max_val.b = max(color.b, max_val.b);
        max_val.a = max(color.a, max_val.a);
        
        // increment the ray sampling position
        sampling_pos  += ray_increment;

        // update the loop termination condition
        inside_volume  = inside_volume_bounds(sampling_pos);
    }

    dst = max_val;
#endif 
    
#if TASK == 11
	
	vec4 sum_val = vec4(0.0, 0.0, 0.0, 0.0);
	int times = 0;
    // the traversal loop,
    // termination when the sampling position is outside volume boundarys
    // another termination condition for early ray termination is added
    while (inside_volume)
    {      
        // get sample
        float s = get_sample_data(sampling_pos);

        // apply the transfer functions to retrieve color and opacity
        vec4 color = texture(transfer_texture, vec2(s, s));

     	// sum up
        sum_val += color;
        
        // increment the ray sampling position
        sampling_pos  += ray_increment;

        // update the loop termination condition
        inside_volume  = inside_volume_bounds(sampling_pos);

        times++;
    }

    // return average
    dst = sum_val/times;
#endif
    
#if TASK == 12 || TASK == 13
	vec3 old_pos = vec3(0.0, 0.0, 0.0);
	float old_s = 0.0;
    bool breaking = false;
    bool go_light = false;
    // the traversal loop,
    // termination when the sampling position is outside volume boundarys
    // another termination condition for early ray termination is added
    while (inside_volume)
    {
        // get samples (density values)
        float s = get_sample_data(sampling_pos);

        #if TASK == 12			// basic
        	// testing if sample is the correct one
        	// Problem: gets only the same or higher density values (in this case enough)
        	if (s - iso_value >= 0) {
        		// apply the transfer functions to retrieve color and opacity
        		dst = texture(transfer_texture, vec2(s, s));
       			breaking = true;			// break while-loop
                go_light = true;
        	}
        #endif

        #if TASK == 13 			// Binary Search
        	if (s - iso_value >= 0 && old_s - iso_value < 0 && old_s != 0.0) {
        		vec3 new_coordinates = binary_search(old_pos, sampling_pos, 0.00001, 10000);
        		float new_s = get_sample_data(new_coordinates);
        		// apply the transfer functions to retrieve color and opacity
        		dst = texture(transfer_texture, vec2(new_s, new_s));
                sampling_pos = new_coordinates;
                breaking = true;
                go_light = true;
            }
        #endif

        // LIGHT (Phong Shading)
        #if ENABLE_LIGHTNING == 1
        if (go_light == true) {
            // vec3 n = get_gradient(sampling_pos);
            // n = think_positiv(n);
            // color = vec4(n.x, n.y, n.z, 1.0);    // map normal to color

            dst = light_it_up(sampling_pos, 2);

            // SHADOWS
            #if ENABLE_SHADOWING == 1
                dst = shadow_it(sampling_pos, dst, false);
            #endif
        }
        #endif

        if (breaking == true) {
            break;      // break while-loop
        }

		// old values for Binary Search
		old_pos = sampling_pos;
        old_s = get_sample_data(sampling_pos);

		// increment the ray sampling position
        sampling_pos += ray_increment;

        // update the loop termination condition
        inside_volume = inside_volume_bounds(sampling_pos);
    }
#endif 

#if TASK == 31
    float trans = 1.0;
    float s = get_sample_data(sampling_pos);
    vec4 color = texture(transfer_texture, vec2(s, s));
    dst.xyz = color.xyz * color.w;
    float old_opac = color.w;
    vec3 inten = vec3(0,0,0);

    #if ENABLE_FRONTBACK == 1
        sampling_pos += ray_increment;
    #else
        while (inside_volume) {
            sampling_pos += ray_increment;
            inside_volume = inside_volume_bounds(sampling_pos);
        }
        sampling_pos -= ray_increment;
        inside_volume = inside_volume_bounds(sampling_pos);
    #endif
    
    while (inside_volume)
    {
        s = get_sample_data(sampling_pos);
        color = texture(transfer_texture, vec2(s, s));
        trans = trans * (1.0 - old_opac);

        // Add Shading
        #if ENABLE_LIGHTNING == 1
            color = vec4(light_it_up(sampling_pos, 3).xyz, color.w);
        #endif

        // Opacity Correction
        #if ENABLE_OPACITY_CORRECTION == 1
            color.w = 1.0 - pow((1.0 - color.w), 200 * sampling_distance / sampling_distance_ref);
        #endif

        // Front-To-Back
        #if ENABLE_FRONTBACK == 1
            dst.xyz = dst.xyz + trans * color.xyz * color.w;
            old_opac = color.w;
            if (trans < 0.0001) {
                break;
            }
            sampling_pos += ray_increment;
        #else
            inten = color.xyz * color.w + inten * (1.0 - color.w);
            sampling_pos -= ray_increment;
        #endif

        inside_volume = inside_volume_bounds(sampling_pos);
    }

    #if ENABLE_FRONTBACK == 1
        dst = vec4(dst.xyz, 1.0 - trans);
    #else
        dst = vec4(inten, 1.0);
    #endif

#endif 

    // return the calculated color value
    FragColor = dst;
}