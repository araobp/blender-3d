import bpy
import math
import json
import collections
import traceback
from math import pi
from bpy.props import StringProperty
from mathutils import Euler, Matrix, Quaternion, Vector
from rna_prop_ui import rna_idprop_quote_path

rig_id = "mw6ywfy2f2374456"


############################
## Math utility functions ##
############################

def perpendicular_vector(v):
    """ Returns a vector that is perpendicular to the one given.
        The returned vector is _not_ guaranteed to be normalized.
    """
    # Create a vector that is not aligned with v.
    # It doesn't matter what vector.  Just any vector
    # that's guaranteed to not be pointing in the same
    # direction.
    if abs(v[0]) < abs(v[1]):
        tv = Vector((1,0,0))
    else:
        tv = Vector((0,1,0))

    # Use cross product to generate a vector perpendicular to
    # both tv and (more importantly) v.
    return v.cross(tv)


def rotation_difference(mat1, mat2):
    """ Returns the shortest-path rotational difference between two
        matrices.
    """
    q1 = mat1.to_quaternion()
    q2 = mat2.to_quaternion()
    angle = math.acos(min(1,max(-1,q1.dot(q2)))) * 2
    if angle > pi:
        angle = -angle + (2*pi)
    return angle

def find_min_range(f,start_angle,delta=pi/8):
    """ finds the range where lies the minimum of function f applied on bone_ik and bone_fk
        at a certain angle.
    """
    angle = start_angle
    while (angle > (start_angle - 2*pi)) and (angle < (start_angle + 2*pi)):
        l_dist = f(angle-delta)
        c_dist = f(angle)
        r_dist = f(angle+delta)
        if min((l_dist,c_dist,r_dist)) == c_dist:
            return (angle-delta,angle+delta)
        else:
            angle=angle+delta

def ternarySearch(f, left, right, absolutePrecision):
    """
    Find minimum of uni-modal function f() within [left, right]
    To find the maximum, revert the if/else statement or revert the comparison.
    """
    while True:
        #left and right are the current bounds; the maximum is between them
        if abs(right - left) < absolutePrecision:
            return (left + right)/2

        leftThird = left + (right - left)/3
        rightThird = right - (right - left)/3

        if f(leftThird) > f(rightThird):
            left = leftThird
        else:
            right = rightThird

######################
## Keyframing tools ##
######################

def get_keying_flags(context):
    "Retrieve the general keyframing flags from user preferences."
    prefs = context.preferences
    ts = context.scene.tool_settings
    flags = set()
    # Not adding INSERTKEY_VISUAL
    if prefs.edit.use_keyframe_insert_needed:
        flags.add('INSERTKEY_NEEDED')
    if prefs.edit.use_insertkey_xyz_to_rgb:
        flags.add('INSERTKEY_XYZ_TO_RGB')
    if ts.use_keyframe_cycle_aware:
        flags.add('INSERTKEY_CYCLE_AWARE')
    return flags

def get_autokey_flags(context, ignore_keyingset=False):
    "Retrieve the Auto Keyframe flags, or None if disabled."
    ts = context.scene.tool_settings
    if ts.use_keyframe_insert_auto and (ignore_keyingset or not ts.use_keyframe_insert_keyingset):
        flags = get_keying_flags(context)
        if context.preferences.edit.use_keyframe_insert_available:
            flags.add('INSERTKEY_AVAILABLE')
        if ts.auto_keying_mode == 'REPLACE_KEYS':
            flags.add('INSERTKEY_REPLACE')
        return flags
    else:
        return None

def add_flags_if_set(base, new_flags):
    "Add more flags if base is not None."
    if base is None:
        return None
    else:
        return base | new_flags

def get_4d_rot_lock(bone):
    "Retrieve the lock status for 4D rotation."
    if bone.lock_rotations_4d:
        return [bone.lock_rotation_w, *bone.lock_rotation]
    else:
        return [all(bone.lock_rotation)] * 4

def keyframe_transform_properties(obj, bone_name, keyflags, *,
                                  ignore_locks=False, no_loc=False, no_rot=False, no_scale=False):
    "Keyframe transformation properties, taking flags and mode into account, and avoiding keying locked channels."
    bone = obj.pose.bones[bone_name]

    def keyframe_channels(prop, locks):
        if ignore_locks or not all(locks):
            if ignore_locks or not any(locks):
                bone.keyframe_insert(prop, group=bone_name, options=keyflags)
            else:
                for i, lock in enumerate(locks):
                    if not lock:
                        bone.keyframe_insert(prop, index=i, group=bone_name, options=keyflags)

    if not (no_loc or bone.bone.use_connect):
        keyframe_channels('location', bone.lock_location)

    if not no_rot:
        if bone.rotation_mode == 'QUATERNION':
            keyframe_channels('rotation_quaternion', get_4d_rot_lock(bone))
        elif bone.rotation_mode == 'AXIS_ANGLE':
            keyframe_channels('rotation_axis_angle', get_4d_rot_lock(bone))
        else:
            keyframe_channels('rotation_euler', bone.lock_rotation)

    if not no_scale:
        keyframe_channels('scale', bone.lock_scale)

######################
## Constraint tools ##
######################

def get_constraint_target_matrix(con):
    target = con.target
    if target:
        if target.type == 'ARMATURE' and con.subtarget:
            if con.subtarget in target.pose.bones:
                bone = target.pose.bones[con.subtarget]
                return target.convert_space(
                    pose_bone=bone, matrix=bone.matrix, from_space='POSE', to_space=con.target_space)
        else:
            return target.convert_space(matrix=target.matrix_world, from_space='WORLD', to_space=con.target_space)
    return Matrix.Identity(4)

def undo_copy_scale_with_offset(obj, bone, con, old_matrix):
    "Undo the effects of Copy Scale with Offset constraint on a bone matrix."
    inf = con.influence

    if con.mute or inf == 0 or not con.is_valid or not con.use_offset or con.use_add:
        return old_matrix

    tgt_matrix = get_constraint_target_matrix(con)
    tgt_scale = tgt_matrix.to_scale()
    use = [con.use_x, con.use_y, con.use_z]

    if con.use_make_uniform:
        if con.use_x and con.use_y and con.use_z:
            total = tgt_matrix.determinant()
        else:
            total = 1
            for i, use in enumerate(use):
                if use:
                    total *= tgt_scale[i]

        tgt_scale = [abs(total)**(1./3.)]*3
    else:
        for i, use in enumerate(use):
            if not use:
                tgt_scale[i] = 1

    scale_delta = [
        1 / (1 + (math.pow(x, con.power) - 1) * inf)
        for x in tgt_scale
    ]

    return old_matrix @ Matrix.Diagonal([*scale_delta, 1])

def undo_copy_scale_constraints(obj, bone, matrix):
    "Undo the effects of all Copy Scale with Offset constraints on a bone matrix."
    for con in reversed(bone.constraints):
        if con.type == 'COPY_SCALE':
            matrix = undo_copy_scale_with_offset(obj, bone, con, matrix)
    return matrix

###############################
## Assign and keyframe tools ##
###############################

def set_custom_property_value(obj, bone_name, prop, value, *, keyflags=None):
    "Assign the value of a custom property, and optionally keyframe it."
    from rna_prop_ui import rna_idprop_ui_prop_update
    bone = obj.pose.bones[bone_name]
    bone[prop] = value
    rna_idprop_ui_prop_update(bone, prop)
    if keyflags is not None:
        bone.keyframe_insert(rna_idprop_quote_path(prop), group=bone.name, options=keyflags)

def get_transform_matrix(obj, bone_name, *, space='POSE', with_constraints=True):
    "Retrieve the matrix of the bone before or after constraints in the given space."
    bone = obj.pose.bones[bone_name]
    if with_constraints:
        return obj.convert_space(pose_bone=bone, matrix=bone.matrix, from_space='POSE', to_space=space)
    else:
        return obj.convert_space(pose_bone=bone, matrix=bone.matrix_basis, from_space='LOCAL', to_space=space)

def get_chain_transform_matrices(obj, bone_names, **options):
    return [get_transform_matrix(obj, name, **options) for name in bone_names]

def set_transform_from_matrix(obj, bone_name, matrix, *, space='POSE', undo_copy_scale=False,
                              ignore_locks=False, no_loc=False, no_rot=False, no_scale=False, keyflags=None):
    """Apply the matrix to the transformation of the bone, taking locked channels, mode and certain
    constraints into account, and optionally keyframe it."""
    bone = obj.pose.bones[bone_name]

    def restore_channels(prop, old_vec, locks, extra_lock):
        if extra_lock or (not ignore_locks and all(locks)):
            setattr(bone, prop, old_vec)
        else:
            if not ignore_locks and any(locks):
                new_vec = Vector(getattr(bone, prop))

                for i, lock in enumerate(locks):
                    if lock:
                        new_vec[i] = old_vec[i]

                setattr(bone, prop, new_vec)

    # Save the old values of the properties
    old_loc = Vector(bone.location)
    old_rot_euler = Vector(bone.rotation_euler)
    old_rot_quat = Vector(bone.rotation_quaternion)
    old_rot_axis = Vector(bone.rotation_axis_angle)
    old_scale = Vector(bone.scale)

    # Compute and assign the local matrix
    if space != 'LOCAL':
        matrix = obj.convert_space(pose_bone=bone, matrix=matrix, from_space=space, to_space='LOCAL')

    if undo_copy_scale:
        matrix = undo_copy_scale_constraints(obj, bone, matrix)

    bone.matrix_basis = matrix

    # Restore locked properties
    restore_channels('location', old_loc, bone.lock_location, no_loc or bone.bone.use_connect)

    if bone.rotation_mode == 'QUATERNION':
        restore_channels('rotation_quaternion', old_rot_quat, get_4d_rot_lock(bone), no_rot)
        bone.rotation_axis_angle = old_rot_axis
        bone.rotation_euler = old_rot_euler
    elif bone.rotation_mode == 'AXIS_ANGLE':
        bone.rotation_quaternion = old_rot_quat
        restore_channels('rotation_axis_angle', old_rot_axis, get_4d_rot_lock(bone), no_rot)
        bone.rotation_euler = old_rot_euler
    else:
        bone.rotation_quaternion = old_rot_quat
        bone.rotation_axis_angle = old_rot_axis
        restore_channels('rotation_euler', old_rot_euler, bone.lock_rotation, no_rot)

    restore_channels('scale', old_scale, bone.lock_scale, no_scale)

    # Keyframe properties
    if keyflags is not None:
        keyframe_transform_properties(
            obj, bone_name, keyflags, ignore_locks=ignore_locks,
            no_loc=no_loc, no_rot=no_rot, no_scale=no_scale
        )

def set_chain_transforms_from_matrices(context, obj, bone_names, matrices, **options):
    for bone, matrix in zip(bone_names, matrices):
        set_transform_from_matrix(obj, bone, matrix, **options)
        context.view_layer.update()


###########################
## Animation curve tools ##
###########################

def flatten_curve_set(curves):
    "Iterate over all FCurves inside a set of nested lists and dictionaries."
    if curves is None:
        pass
    elif isinstance(curves, bpy.types.FCurve):
        yield curves
    elif isinstance(curves, dict):
        for sub in curves.values():
            yield from flatten_curve_set(sub)
    else:
        for sub in curves:
            yield from flatten_curve_set(sub)

def flatten_curve_key_set(curves, key_range=None):
    "Iterate over all keys of the given fcurves in the specified range."
    for curve in flatten_curve_set(curves):
        for key in curve.keyframe_points:
            if key_range is None or key_range[0] <= key.co[0] <= key_range[1]:
                yield key

def get_curve_frame_set(curves, key_range=None):
    "Compute a set of all time values with existing keys in the given curves and range."
    return set(key.co[0] for key in flatten_curve_key_set(curves, key_range))

def set_curve_key_interpolation(curves, ipo, key_range=None):
    "Assign the given interpolation value to all curve keys in range."
    for key in flatten_curve_key_set(curves, key_range):
        key.interpolation = ipo

def delete_curve_keys_in_range(curves, key_range=None):
    "Delete all keys of the given curves within the given range."
    for curve in flatten_curve_set(curves):
        points = curve.keyframe_points
        for i in range(len(points), 0, -1):
            key = points[i - 1]
            if key_range is None or key_range[0] <= key.co[0] <= key_range[1]:
                points.remove(key, fast=True)
        curve.update()

def nla_tweak_to_scene(anim_data, frames, invert=False):
    "Convert a frame value or list between scene and tweaked NLA strip time."
    if frames is None:
        return None
    elif anim_data is None or not anim_data.use_tweak_mode:
        return frames
    elif isinstance(frames, (int, float)):
        return anim_data.nla_tweak_strip_time_to_scene(frames, invert=invert)
    else:
        return type(frames)(
            anim_data.nla_tweak_strip_time_to_scene(v, invert=invert) for v in frames
        )

def find_action(action):
    if isinstance(action, bpy.types.Object):
        action = action.animation_data
    if isinstance(action, bpy.types.AnimData):
        action = action.action
    if isinstance(action, bpy.types.Action):
        return action
    else:
        return None

def clean_action_empty_curves(action):
    "Delete completely empty curves from the given action."
    action = find_action(action)
    for curve in list(action.fcurves):
        if curve.is_empty:
            action.fcurves.remove(curve)
    action.update_tag()

TRANSFORM_PROPS_LOCATION = frozenset(['location'])
TRANSFORM_PROPS_ROTATION = frozenset(['rotation_euler', 'rotation_quaternion', 'rotation_axis_angle'])
TRANSFORM_PROPS_SCALE = frozenset(['scale'])
TRANSFORM_PROPS_ALL = frozenset(TRANSFORM_PROPS_LOCATION | TRANSFORM_PROPS_ROTATION | TRANSFORM_PROPS_SCALE)

def transform_props_with_locks(lock_location, lock_rotation, lock_scale):
    props = set()
    if not lock_location:
        props |= TRANSFORM_PROPS_LOCATION
    if not lock_rotation:
        props |= TRANSFORM_PROPS_ROTATION
    if not lock_scale:
        props |= TRANSFORM_PROPS_SCALE
    return props

class FCurveTable(object):
    "Table for efficient lookup of FCurves by properties."

    def __init__(self):
        self.curve_map = collections.defaultdict(dict)

    def index_curves(self, curves):
        for curve in curves:
            index = curve.array_index
            if index < 0:
                index = 0
            self.curve_map[curve.data_path][index] = curve

    def get_prop_curves(self, ptr, prop_path):
        "Returns a dictionary from array index to curve for the given property, or Null."
        return self.curve_map.get(ptr.path_from_id(prop_path))

    def list_all_prop_curves(self, ptr_set, path_set):
        "Iterates over all FCurves matching the given object(s) and properties."
        if isinstance(ptr_set, bpy.types.bpy_struct):
            ptr_set = [ptr_set]
        for ptr in ptr_set:
            for path in path_set:
                curves = self.get_prop_curves(ptr, path)
                if curves:
                    yield from curves.values()

    def get_custom_prop_curves(self, ptr, prop):
        return self.get_prop_curves(ptr, rna_idprop_quote_path(prop))

class ActionCurveTable(FCurveTable):
    "Table for efficient lookup of Action FCurves by properties."

    def __init__(self, action):
        super().__init__()
        self.action = find_action(action)
        if self.action:
            self.index_curves(self.action.fcurves)

class DriverCurveTable(FCurveTable):
    "Table for efficient lookup of Driver FCurves by properties."

    def __init__(self, object):
        super().__init__()
        self.anim_data = object.animation_data
        if self.anim_data:
            self.index_curves(self.anim_data.drivers)


##################################
# Common bake operator settings ##
##################################

bpy.types.WindowManager.rigify_transfer_use_all_keys = bpy.props.BoolProperty(
    name="Bake All Keyed Frames",
    description="Bake on every frame that has a key for any of the bones, as opposed to just the relevant ones",
    default=False
)
bpy.types.WindowManager.rigify_transfer_use_frame_range = bpy.props.BoolProperty(
    name="Limit Frame Range", description="Only bake keyframes in a certain frame range", default=False
)
bpy.types.WindowManager.rigify_transfer_start_frame = bpy.props.IntProperty(
    name="Start", description="First frame to transfer", default=0, min=0
)
bpy.types.WindowManager.rigify_transfer_end_frame = bpy.props.IntProperty(
    name="End", description="Last frame to transfer", default=0, min=0
)

class RIGIFY_OT_get_frame_range(bpy.types.Operator):
    bl_idname = "rigify.get_frame_range" + ('_'+rig_id if rig_id else '')
    bl_label = "Get Frame Range"
    bl_description = "Set start and end frame from scene"
    bl_options = {'INTERNAL'}

    def execute(self, context):
        scn = context.scene
        id_store = context.window_manager
        id_store.rigify_transfer_start_frame = scn.frame_start
        id_store.rigify_transfer_end_frame = scn.frame_end
        return {'FINISHED'}

    @staticmethod
    def get_range(context):
        id_store = context.window_manager
        if not id_store.rigify_transfer_use_frame_range:
            return None
        else:
            return (id_store.rigify_transfer_start_frame, id_store.rigify_transfer_end_frame)

    @classmethod
    def draw_range_ui(self, context, layout):
        id_store = context.window_manager

        row = layout.row(align=True)
        row.prop(id_store, 'rigify_transfer_use_frame_range', icon='PREVIEW_RANGE', text='')

        row = row.row(align=True)
        row.active = id_store.rigify_transfer_use_frame_range
        row.prop(id_store, 'rigify_transfer_start_frame')
        row.prop(id_store, 'rigify_transfer_end_frame')
        row.operator(self.bl_idname, icon='TIME', text='')

#######################################
# Keyframe baking operator framework ##
#######################################

class RigifyOperatorMixinBase:
    bl_options = {'UNDO', 'INTERNAL'}

    def init_invoke(self, context):
        "Override to initialize the operator before invoke."

    def init_execute(self, context):
        "Override to initialize the operator before execute."

    def before_save_state(self, context, rig):
        "Override to prepare for saving state."

    def after_save_state(self, context, rig):
        "Override to undo before_save_state."


class RigifyBakeKeyframesMixin(RigifyOperatorMixinBase):
    """Basic framework for an operator that updates a set of keyed frames."""

    # Utilities
    def nla_from_raw(self, frames):
        "Convert frame(s) from inner action time to scene time."
        return nla_tweak_to_scene(self.bake_anim, frames)

    def nla_to_raw(self, frames):
        "Convert frame(s) from scene time to inner action time."
        return nla_tweak_to_scene(self.bake_anim, frames, invert=True)

    def bake_get_bone(self, bone_name):
        "Get pose bone by name."
        return self.bake_rig.pose.bones[bone_name]

    def bake_get_bones(self, bone_names):
        "Get multiple pose bones by name."
        if isinstance(bone_names, (list, set)):
            return [self.bake_get_bone(name) for name in bone_names]
        else:
            return self.bake_get_bone(bone_names)

    def bake_get_all_bone_curves(self, bone_names, props):
        "Get a list of all curves for the specified properties of the specified bones."
        return list(self.bake_curve_table.list_all_prop_curves(self.bake_get_bones(bone_names), props))

    def bake_get_all_bone_custom_prop_curves(self, bone_names, props):
        "Get a list of all curves for the specified custom properties of the specified bones."
        return self.bake_get_all_bone_curves(bone_names, [rna_idprop_quote_path(p) for p in props])

    def bake_get_bone_prop_curves(self, bone_name, prop):
        "Get an index to curve dict for the specified property of the specified bone."
        return self.bake_curve_table.get_prop_curves(self.bake_get_bone(bone_name), prop)

    def bake_get_bone_custom_prop_curves(self, bone_name, prop):
        "Get an index to curve dict for the specified custom property of the specified bone."
        return self.bake_curve_table.get_custom_prop_curves(self.bake_get_bone(bone_name), prop)

    def bake_add_curve_frames(self, curves):
        "Register frames keyed in the specified curves for baking."
        self.bake_frames_raw |= get_curve_frame_set(curves, self.bake_frame_range_raw)

    def bake_add_bone_frames(self, bone_names, props):
        "Register frames keyed for the specified properties of the specified bones for baking."
        curves = self.bake_get_all_bone_curves(bone_names, props)
        self.bake_add_curve_frames(curves)
        return curves

    def bake_replace_custom_prop_keys_constant(self, bone, prop, new_value):
        "If the property is keyframed, delete keys in bake range and re-key as Constant."
        prop_curves = self.bake_get_bone_custom_prop_curves(bone, prop)

        if prop_curves and 0 in prop_curves:
            range_raw = self.nla_to_raw(self.get_bake_range())
            delete_curve_keys_in_range(prop_curves, range_raw)
            set_custom_property_value(self.bake_rig, bone, prop, new_value, keyflags={'INSERTKEY_AVAILABLE'})
            set_curve_key_interpolation(prop_curves, 'CONSTANT', range_raw)

    # Default behavior implementation
    def bake_init(self, context):
        self.bake_rig = context.active_object
        self.bake_anim = self.bake_rig.animation_data
        self.bake_frame_range = RIGIFY_OT_get_frame_range.get_range(context)
        self.bake_frame_range_raw = self.nla_to_raw(self.bake_frame_range)
        self.bake_curve_table = ActionCurveTable(self.bake_rig)
        self.bake_current_frame = context.scene.frame_current
        self.bake_frames_raw = set()
        self.bake_state = dict()

        self.keyflags = get_keying_flags(context)
        self.keyflags_switch = None

        if context.window_manager.rigify_transfer_use_all_keys:
            self.bake_add_curve_frames(self.bake_curve_table.curve_map)

    def bake_add_frames_done(self):
        "Computes and sets the final set of frames to bake."
        frames = self.nla_from_raw(self.bake_frames_raw)
        self.bake_frames = sorted(set(map(round, frames)))

    def is_bake_empty(self):
        return len(self.bake_frames_raw) == 0

    def report_bake_empty(self):
        self.bake_add_frames_done()
        if self.is_bake_empty():
            self.report({'WARNING'}, 'No keys to bake.')
            return True
        return False

    def get_bake_range(self):
        "Returns the frame range that is being baked."
        if self.bake_frame_range:
            return self.bake_frame_range
        else:
            frames = self.bake_frames
            return (frames[0], frames[-1])

    def get_bake_range_pair(self):
        "Returns the frame range that is being baked, both in scene and action time."
        range = self.get_bake_range()
        return range, self.nla_to_raw(range)

    def bake_save_state(self, context):
        "Scans frames and collects data for baking before changing anything."
        rig = self.bake_rig
        scene = context.scene
        saved_state = self.bake_state

        try:
            self.before_save_state(context, rig)

            for frame in self.bake_frames:
                scene.frame_set(frame)
                saved_state[frame] = self.save_frame_state(context, rig)

        finally:
            self.after_save_state(context, rig)

    def bake_clean_curves_in_range(self, context, curves):
        "Deletes all keys from the given curves in the bake range."
        range, range_raw = self.get_bake_range_pair()

        context.scene.frame_set(range[0])
        delete_curve_keys_in_range(curves, range_raw)

        return range, range_raw

    def bake_apply_state(self, context):
        "Scans frames and applies the baking operation."
        rig = self.bake_rig
        scene = context.scene
        saved_state = self.bake_state

        for frame in self.bake_frames:
            scene.frame_set(frame)
            self.apply_frame_state(context, rig, saved_state.get(frame))

        clean_action_empty_curves(self.bake_rig)
        scene.frame_set(self.bake_current_frame)

    @staticmethod
    def draw_common_bake_ui(context, layout):
        layout.prop(context.window_manager, 'rigify_transfer_use_all_keys')

        RIGIFY_OT_get_frame_range.draw_range_ui(context, layout)

    @classmethod
    def poll(cls, context):
        return find_action(context.active_object) is not None

    def execute_scan_curves(self, context, obj):
        "Override to register frames to be baked, and return curves that should be cleared."
        raise NotImplementedError()

    def execute_before_apply(self, context, obj, range, range_raw):
        "Override to execute code one time before the bake apply frame scan."
        pass

    def execute(self, context):
        self.init_execute(context)
        self.bake_init(context)

        curves = self.execute_scan_curves(context, self.bake_rig)

        if self.report_bake_empty():
            return {'CANCELLED'}

        try:
            self.bake_save_state(context)

            range, range_raw = self.bake_clean_curves_in_range(context, curves)

            self.execute_before_apply(context, self.bake_rig, range, range_raw)

            self.bake_apply_state(context)

        except Exception as e:
            traceback.print_exc()
            self.report({'ERROR'}, 'Exception: ' + str(e))

        return {'FINISHED'}

    def invoke(self, context, event):
        self.init_invoke(context)

        if hasattr(self, 'draw'):
            return context.window_manager.invoke_props_dialog(self)
        else:
            return context.window_manager.invoke_confirm(self, event)


class RigifySingleUpdateMixin(RigifyOperatorMixinBase):
    """Basic framework for an operator that updates only the current frame."""

    def execute(self, context):
        self.init_execute(context)
        obj = context.active_object
        self.keyflags = get_autokey_flags(context, ignore_keyingset=True)
        self.keyflags_switch = add_flags_if_set(self.keyflags, {'INSERTKEY_AVAILABLE'})

        try:
            try:
                self.before_save_state(context, obj)
                state = self.save_frame_state(context, obj)
            finally:
                self.after_save_state(context, obj)

            self.apply_frame_state(context, obj, state)

        except Exception as e:
            traceback.print_exc()
            self.report({'ERROR'}, 'Exception: ' + str(e))

        return {'FINISHED'}

    def invoke(self, context, event):
        self.init_invoke(context)

        if hasattr(self, 'draw'):
            return context.window_manager.invoke_props_popup(self, event)
        else:
            return self.execute(context)


#############################
## Generic Snap (FK to IK) ##
#############################

class RigifyGenericSnapBase:
    input_bones:   StringProperty(name="Input Chain")
    output_bones:  StringProperty(name="Output Chain")
    ctrl_bones:    StringProperty(name="Input Controls")

    tooltip:         StringProperty(name="Tooltip", default="FK to IK")
    locks:           bpy.props.BoolVectorProperty(name="Locked", size=3, default=[False,False,False])
    undo_copy_scale: bpy.props.BoolProperty(name="Undo Copy Scale", default=False)

    def init_execute(self, context):
        self.input_bone_list = json.loads(self.input_bones)
        self.output_bone_list = json.loads(self.output_bones)
        self.ctrl_bone_list = json.loads(self.ctrl_bones)

    def save_frame_state(self, context, obj):
        return get_chain_transform_matrices(obj, self.input_bone_list)

    def apply_frame_state(self, context, obj, matrices):
        set_chain_transforms_from_matrices(
            context, obj, self.output_bone_list, matrices,
            undo_copy_scale=self.undo_copy_scale, keyflags=self.keyflags,
            no_loc=self.locks[0], no_rot=self.locks[1], no_scale=self.locks[2],
        )

class POSE_OT_rigify_generic_snap(RigifyGenericSnapBase, RigifySingleUpdateMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_generic_snap_" + rig_id
    bl_label = "Snap Bones"
    bl_description = "Snap on the current frame"

    @classmethod
    def description(cls, context, props):
        return "Snap " + props.tooltip + " on the current frame"

class POSE_OT_rigify_generic_snap_bake(RigifyGenericSnapBase, RigifyBakeKeyframesMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_generic_snap_bake_" + rig_id
    bl_label = "Apply Snap To Keyframes"
    bl_description = "Apply snap to keyframes"

    @classmethod
    def description(cls, context, props):
        return "Apply snap " + props.tooltip + " to keyframes"

    def execute_scan_curves(self, context, obj):
        props = transform_props_with_locks(*self.locks)
        self.bake_add_bone_frames(self.ctrl_bone_list, TRANSFORM_PROPS_ALL)
        return self.bake_get_all_bone_curves(self.output_bone_list, props)


#############################
## Generic Clear Keyframes ##
#############################

class POSE_OT_rigify_clear_keyframes(bpy.types.Operator):
    bl_idname = "pose.rigify_clear_keyframes_" + rig_id
    bl_label = "Clear Keyframes And Transformation"
    bl_options = {'UNDO', 'INTERNAL'}
    bl_description = "Remove all keyframes for the relevant bones and reset transformation"

    bones: StringProperty(name="Bone List")

    @classmethod
    def poll(cls, context):
        return find_action(context.active_object) is not None

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        obj = context.active_object
        bone_list = [ obj.pose.bones[name] for name in json.loads(self.bones) ]

        curve_table = ActionCurveTable(context.active_object)
        curves = list(curve_table.list_all_prop_curves(bone_list, TRANSFORM_PROPS_ALL))

        key_range = RIGIFY_OT_get_frame_range.get_range(context)
        range_raw = nla_tweak_to_scene(obj.animation_data, key_range, invert=True)
        delete_curve_keys_in_range(curves, range_raw)

        for bone in bone_list:
            bone.location = bone.rotation_euler = (0,0,0)
            bone.rotation_quaternion = (1,0,0,0)
            bone.rotation_axis_angle = (0,0,1,0)
            bone.scale = (1,1,1)

        clean_action_empty_curves(obj)
        obj.update_tag(refresh={'TIME'})
        return {'FINISHED'}


#########################################
## "Visual Transform" helper functions ##
#########################################

def get_pose_matrix_in_other_space(mat, pose_bone):
    """ Returns the transform matrix relative to pose_bone's current
        transform space.  In other words, presuming that mat is in
        armature space, slapping the returned matrix onto pose_bone
        should give it the armature-space transforms of mat.
    """
    return pose_bone.id_data.convert_space(matrix=mat, pose_bone=pose_bone, from_space='POSE', to_space='LOCAL')


def convert_pose_matrix_via_rest_delta(mat, from_bone, to_bone):
    """Convert pose of one bone to another bone, preserving the rest pose difference between them."""
    return mat @ from_bone.bone.matrix_local.inverted() @ to_bone.bone.matrix_local


def convert_pose_matrix_via_pose_delta(mat, from_bone, to_bone):
    """Convert pose of one bone to another bone, preserving the current pose difference between them."""
    return mat @ from_bone.matrix.inverted() @ to_bone.matrix


def get_local_pose_matrix(pose_bone):
    """ Returns the local transform matrix of the given pose bone.
    """
    return get_pose_matrix_in_other_space(pose_bone.matrix, pose_bone)


def set_pose_translation(pose_bone, mat):
    """ Sets the pose bone's translation to the same translation as the given matrix.
        Matrix should be given in bone's local space.
    """
    pose_bone.location = mat.to_translation()


def set_pose_rotation(pose_bone, mat):
    """ Sets the pose bone's rotation to the same rotation as the given matrix.
        Matrix should be given in bone's local space.
    """
    q = mat.to_quaternion()

    if pose_bone.rotation_mode == 'QUATERNION':
        pose_bone.rotation_quaternion = q
    elif pose_bone.rotation_mode == 'AXIS_ANGLE':
        pose_bone.rotation_axis_angle[0] = q.angle
        pose_bone.rotation_axis_angle[1] = q.axis[0]
        pose_bone.rotation_axis_angle[2] = q.axis[1]
        pose_bone.rotation_axis_angle[3] = q.axis[2]
    else:
        pose_bone.rotation_euler = q.to_euler(pose_bone.rotation_mode)


def set_pose_scale(pose_bone, mat):
    """ Sets the pose bone's scale to the same scale as the given matrix.
        Matrix should be given in bone's local space.
    """
    pose_bone.scale = mat.to_scale()


def match_pose_translation(pose_bone, target_bone):
    """ Matches pose_bone's visual translation to target_bone's visual
        translation.
        This function assumes you are in pose mode on the relevant armature.
    """
    mat = get_pose_matrix_in_other_space(target_bone.matrix, pose_bone)
    set_pose_translation(pose_bone, mat)


def match_pose_rotation(pose_bone, target_bone):
    """ Matches pose_bone's visual rotation to target_bone's visual
        rotation.
        This function assumes you are in pose mode on the relevant armature.
    """
    mat = get_pose_matrix_in_other_space(target_bone.matrix, pose_bone)
    set_pose_rotation(pose_bone, mat)


def match_pose_scale(pose_bone, target_bone):
    """ Matches pose_bone's visual scale to target_bone's visual
        scale.
        This function assumes you are in pose mode on the relevant armature.
    """
    mat = get_pose_matrix_in_other_space(target_bone.matrix, pose_bone)
    set_pose_scale(pose_bone, mat)


##############################
## IK/FK snapping functions ##
##############################

def correct_rotation(view_layer, bone_ik, target_matrix, *, ctrl_ik=None):
    """ Corrects the ik rotation in ik2fk snapping functions
    """

    axis = target_matrix.to_3x3().col[1].normalized()
    ctrl_ik = ctrl_ik or bone_ik

    def distance(angle):
        # Rotate the bone and return the actual angle between bones
        ctrl_ik.rotation_euler[1] = angle
        view_layer.update()

        return -(bone_ik.vector.normalized().dot(axis))

    if ctrl_ik.rotation_mode in {'QUATERNION', 'AXIS_ANGLE'}:
        ctrl_ik.rotation_mode = 'ZXY'

    start_angle = ctrl_ik.rotation_euler[1]

    alpha_range = find_min_range(distance, start_angle)
    alpha_min = ternarySearch(distance, alpha_range[0], alpha_range[1], pi / 180)

    ctrl_ik.rotation_euler[1] = alpha_min
    view_layer.update()


def correct_scale(view_layer, bone_ik, target_matrix, *, ctrl_ik=None):
    """ Correct the scale of the base IK bone. """
    input_scale = target_matrix.to_scale()
    ctrl_ik = ctrl_ik or bone_ik

    for i in range(3):
        cur_scale = bone_ik.matrix.to_scale()

        ctrl_ik.scale = [
            v * i / c for v, i, c in zip(bone_ik.scale, input_scale, cur_scale)
        ]

        view_layer.update()

        if all(abs((c - i)/i) < 0.01 for i, c in zip(input_scale, cur_scale)):
            break


def match_pole_target(view_layer, ik_first, ik_last, pole, match_bone_matrix, length):
    """ Places an IK chain's pole target to match ik_first's
        transforms to match_bone.  All bones should be given as pose bones.
        You need to be in pose mode on the relevant armature object.
        ik_first: first bone in the IK chain
        ik_last:  last bone in the IK chain
        pole:  pole target bone for the IK chain
        match_bone:  bone to match ik_first to (probably first bone in a matching FK chain)
        length:  distance pole target should be placed from the chain center
    """
    a = ik_first.matrix.to_translation()
    b = ik_last.matrix.to_translation() + ik_last.vector

    # Vector from the head of ik_first to the
    # tip of ik_last
    ikv = b - a

    # Get a vector perpendicular to ikv
    pv = perpendicular_vector(ikv).normalized() * length

    def set_pole(pvi):
        """ Set pole target's position based on a vector
            from the arm center line.
        """
        # Translate pvi into armature space
        pole_loc = a + (ikv/2) + pvi

        # Set pole target to location
        mat = get_pose_matrix_in_other_space(Matrix.Translation(pole_loc), pole)
        set_pose_translation(pole, mat)

        view_layer.update()

    set_pole(pv)

    # Get the rotation difference between ik_first and match_bone
    angle = rotation_difference(ik_first.matrix, match_bone_matrix)

    # Try compensating for the rotation difference in both directions
    pv1 = Matrix.Rotation(angle, 4, ikv) @ pv
    set_pole(pv1)
    ang1 = rotation_difference(ik_first.matrix, match_bone_matrix)

    pv2 = Matrix.Rotation(-angle, 4, ikv) @ pv
    set_pole(pv2)
    ang2 = rotation_difference(ik_first.matrix, match_bone_matrix)

    # Do the one with the smaller angle
    if ang1 < ang2:
        set_pole(pv1)

##########
## Misc ##
##########

def parse_bone_names(names_string):
    if names_string[0] == '[' and names_string[-1] == ']':
        return eval(names_string)
    else:
        return names_string



########################
## Limb Snap IK to FK ##
########################

class RigifyLimbIk2FkBase:
    prop_bone:    StringProperty(name="Settings Bone")
    pole_prop:    StringProperty(name="Pole target switch", default="pole_vector")
    fk_bones:     StringProperty(name="FK Bone Chain")
    ik_bones:     StringProperty(name="IK Result Bone Chain")
    ctrl_bones:   StringProperty(name="IK Controls")
    tail_bones:   StringProperty(name="Tail IK Controls", default="[]")
    extra_ctrls:  StringProperty(name="Extra IK Controls")

    def init_execute(self, context):
        if self.fk_bones:
            self.fk_bone_list = json.loads(self.fk_bones)
        self.ik_bone_list = json.loads(self.ik_bones)
        self.ctrl_bone_list = json.loads(self.ctrl_bones)
        self.tail_bone_list = json.loads(self.tail_bones)
        self.extra_ctrl_list = json.loads(self.extra_ctrls)

    def get_use_pole(self, obj):
        bone = obj.pose.bones[self.prop_bone]
        return self.pole_prop in bone and bone[self.pole_prop]

    def save_frame_state(self, context, obj):
        return get_chain_transform_matrices(obj, self.fk_bone_list)

    def compute_base_rotation(self, context, ik_bones, ctrl_bones, matrices, use_pole):
        context.view_layer.update()

        if use_pole:
            match_pole_target(
                context.view_layer,
                ik_bones[0], ik_bones[1], ctrl_bones[1], matrices[0],
                (ik_bones[0].length + ik_bones[1].length)
            )

        else:
            correct_rotation(context.view_layer, ik_bones[0], matrices[0], ctrl_ik=ctrl_bones[0])

    def assign_middle_controls(self, context, obj, matrices, ik_bones, ctrl_bones, *, lock=False, keyflags=None):
        for mat, ik, ctrl in reversed(list(zip(matrices[2:-1], ik_bones[2:-1], ctrl_bones[2:-1]))):
            ctrl.bone.use_inherit_rotation = not lock
            ctrl.bone.inherit_scale = 'NONE' if lock else 'FULL'
            context.view_layer.update()
            mat = convert_pose_matrix_via_rest_delta(mat, ik, ctrl)
            set_transform_from_matrix(obj, ctrl.name, mat, keyflags=keyflags)

    def assign_extra_controls(self, context, obj, all_matrices, ik_bones, ctrl_bones):
        for extra in self.extra_ctrl_list:
            set_transform_from_matrix(
                obj, extra, Matrix.Identity(4), space='LOCAL', keyflags=self.keyflags
            )

    def apply_frame_state(self, context, obj, all_matrices):
        ik_bones = [ obj.pose.bones[k] for k in self.ik_bone_list ]
        ctrl_bones = [ obj.pose.bones[k] for k in self.ctrl_bone_list ]
        tail_bones = [ obj.pose.bones[k] for k in self.tail_bone_list ]

        assert len(all_matrices) >= len(ik_bones) + len(tail_bones)

        matrices = all_matrices[0:len(ik_bones)]
        tail_matrices = all_matrices[len(ik_bones):]

        use_pole = self.get_use_pole(obj)

        # Remove foot heel transform, if present
        self.assign_extra_controls(context, obj, all_matrices, ik_bones, ctrl_bones)

        context.view_layer.update()

        # Set the end control position
        end_mat = convert_pose_matrix_via_pose_delta(matrices[-1], ik_bones[-1], ctrl_bones[-1])

        set_transform_from_matrix(
            obj, self.ctrl_bone_list[-1], end_mat, keyflags=self.keyflags,
            undo_copy_scale=True,
        )

        # Set the base bone position
        ctrl_bones[0].matrix_basis = Matrix.Identity(4)

        set_transform_from_matrix(
            obj, self.ctrl_bone_list[0], matrices[0],
            no_scale=True, no_rot=use_pole,
        )

        # Lock middle control transforms (first pass)
        self.assign_middle_controls(context, obj, matrices, ik_bones, ctrl_bones, lock=True)

        # Adjust the base bone state
        self.compute_base_rotation(context, ik_bones, ctrl_bones, matrices, use_pole)

        correct_scale(context.view_layer, ik_bones[0], matrices[0], ctrl_ik=ctrl_bones[0])

        # Assign middle control transforms (final pass)
        self.assign_middle_controls(context, obj, matrices, ik_bones, ctrl_bones, keyflags=self.keyflags)

        # Assign tail control transforms
        for mat, ctrl in zip(tail_matrices, tail_bones):
            context.view_layer.update()
            set_transform_from_matrix(obj, ctrl.name, mat, keyflags=self.keyflags)

        # Keyframe controls
        if self.keyflags is not None:
            if use_pole:
                keyframe_transform_properties(
                    obj, self.ctrl_bone_list[1], self.keyflags,
                    no_rot=True, no_scale=True,
                )

            keyframe_transform_properties(
                obj, self.ctrl_bone_list[0], self.keyflags,
                no_rot=use_pole,
            )

class POSE_OT_rigify_limb_ik2fk(RigifyLimbIk2FkBase, RigifySingleUpdateMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_limb_ik2fk_" + rig_id
    bl_label = "Snap IK->FK"
    bl_description = "Snap the IK chain to FK result"

class POSE_OT_rigify_limb_ik2fk_bake(RigifyLimbIk2FkBase, RigifyBakeKeyframesMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_limb_ik2fk_bake_" + rig_id
    bl_label = "Apply Snap IK->FK To Keyframes"
    bl_description = "Snap the IK chain keyframes to FK result"

    def execute_scan_curves(self, context, obj):
        self.bake_add_bone_frames(self.fk_bone_list, TRANSFORM_PROPS_ALL)
        return self.bake_get_all_bone_curves(self.ctrl_bone_list + self.extra_ctrl_list, TRANSFORM_PROPS_ALL)


####################
## Toggle IK Pole ##
####################

class RigifyLimbTogglePoleBase(RigifyLimbIk2FkBase):
    use_pole: bpy.props.BoolProperty(name="Use Pole Vector")

    def save_frame_state(self, context, obj):
        return get_chain_transform_matrices(obj, self.ik_bone_list)

    def apply_frame_state(self, context, obj, matrices):
        ik_bones = [ obj.pose.bones[k] for k in self.ik_bone_list ]
        ctrl_bones = [ obj.pose.bones[k] for k in self.ctrl_bone_list ]

        # Set the pole property
        set_custom_property_value(
            obj, self.prop_bone, self.pole_prop, int(self.use_pole),
            keyflags=self.keyflags_switch
        )

        # Lock middle control transforms
        self.assign_middle_controls(context, obj, matrices, ik_bones, ctrl_bones, lock=True)

        # Reset the base bone rotation
        set_pose_rotation(ctrl_bones[0], Matrix.Identity(4))

        self.compute_base_rotation(context, ik_bones, ctrl_bones, matrices, self.use_pole)

        # Assign middle control transforms (final pass)
        self.assign_middle_controls(context, obj, matrices, ik_bones, ctrl_bones, keyflags=self.keyflags)

        # Keyframe controls
        if self.keyflags is not None:
            if self.use_pole:
                keyframe_transform_properties(
                    obj, self.ctrl_bone_list[2], self.keyflags,
                    no_rot=True, no_scale=True,
                )
            else:
                keyframe_transform_properties(
                    obj, self.ctrl_bone_list[0], self.keyflags,
                    no_loc=True, no_scale=True,
                )

    def init_invoke(self, context):
        self.use_pole = not bool(context.active_object.pose.bones[self.prop_bone][self.pole_prop])

class POSE_OT_rigify_limb_toggle_pole(RigifyLimbTogglePoleBase, RigifySingleUpdateMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_limb_toggle_pole_" + rig_id
    bl_label = "Toggle Pole"
    bl_description = "Switch the IK chain between pole and rotation"

class POSE_OT_rigify_limb_toggle_pole_bake(RigifyLimbTogglePoleBase, RigifyBakeKeyframesMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_limb_toggle_pole_bake_" + rig_id
    bl_label = "Apply Toggle Pole To Keyframes"
    bl_description = "Switch the IK chain between pole and rotation over a frame range"

    def execute_scan_curves(self, context, obj):
        self.bake_add_bone_frames(self.ctrl_bone_list, TRANSFORM_PROPS_ALL)

        rot_curves = self.bake_get_all_bone_curves(self.ctrl_bone_list[0], TRANSFORM_PROPS_ROTATION)
        pole_curves = self.bake_get_all_bone_curves(self.ctrl_bone_list[2], TRANSFORM_PROPS_LOCATION)
        return rot_curves + pole_curves

    def execute_before_apply(self, context, obj, range, range_raw):
        self.bake_replace_custom_prop_keys_constant(self.prop_bone, self.pole_prop, int(self.use_pole))

    def draw(self, context):
        self.layout.prop(self, 'use_pole')


################################
## Switchable Parent operator ##
################################

class RigifySwitchParentBase:
    bone:         StringProperty(name="Control Bone")
    prop_bone:    StringProperty(name="Property Bone")
    prop_id:      StringProperty(name="Property")
    parent_names: StringProperty(name="Parent Names")
    locks:        bpy.props.BoolVectorProperty(name="Locked", size=3, default=[False,False,False])

    parent_items = [('0','None','None')]

    selected: bpy.props.EnumProperty(
        name='Selected Parent',
        items=lambda s,c: RigifySwitchParentBase.parent_items
    )

    def save_frame_state(self, context, obj):
        return get_transform_matrix(obj, self.bone, with_constraints=False)

    def apply_frame_state(self, context, obj, old_matrix):
        # Change the parent
        set_custom_property_value(
            obj, self.prop_bone, self.prop_id, int(self.selected),
            keyflags=self.keyflags_switch
        )

        context.view_layer.update()

        # Set the transforms to restore position
        set_transform_from_matrix(
            obj, self.bone, old_matrix, keyflags=self.keyflags,
            no_loc=self.locks[0], no_rot=self.locks[1], no_scale=self.locks[2]
        )

    def init_invoke(self, context):
        pose = context.active_object.pose

        if (not pose or not self.parent_names
            or self.bone not in pose.bones
            or self.prop_bone not in pose.bones
            or self.prop_id not in pose.bones[self.prop_bone]):
            self.report({'ERROR'}, "Invalid parameters")
            return {'CANCELLED'}

        parents = json.loads(self.parent_names)
        parent_items = [(str(i), name, name) for i, name in enumerate(parents)]

        RigifySwitchParentBase.parent_items = parent_items

        self.selected = str(pose.bones[self.prop_bone][self.prop_id])


class POSE_OT_rigify_switch_parent(RigifySwitchParentBase, RigifySingleUpdateMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_switch_parent_" + rig_id
    bl_label = "Switch Parent (Keep Transform)"
    bl_options = {'REGISTER', 'UNDO', 'INTERNAL'}
    bl_description = "Switch parent, preserving the bone position and orientation"

    def draw(self, _context):
        col = self.layout.column()
        col.prop(self, 'selected', expand=True)


class POSE_OT_rigify_switch_parent_bake(RigifySwitchParentBase, RigifyBakeKeyframesMixin, bpy.types.Operator):
    bl_idname = "pose.rigify_switch_parent_bake_" + rig_id
    bl_label = "Apply Switch Parent To Keyframes"
    bl_description = "Switch parent over a frame range, adjusting keys to preserve the bone position and orientation"

    def execute_scan_curves(self, context, obj):
        return self.bake_add_bone_frames(self.bone, transform_props_with_locks(*self.locks))

    def execute_before_apply(self, context, obj, range, range_raw):
        self.bake_replace_custom_prop_keys_constant(self.prop_bone, self.prop_id, int(self.selected))

    def draw(self, context):
        self.layout.prop(self, 'selected', text='')


###################
## Rig UI Panels ##
###################

class RigUI(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = "Rig Main Properties"
    bl_idname = "VIEW3D_PT_rig_ui_" + rig_id
    bl_category = 'Item'

    @classmethod
    def poll(self, context):
        if context.mode != 'POSE':
            return False
        try:
            return (context.active_object.data.get("rig_id") == rig_id)
        except (AttributeError, KeyError, TypeError):
            return False

    def draw(self, context):
        layout = self.layout
        pose_bones = context.active_object.pose.bones
        try:
            selected_bones = set(bone.name for bone in context.selected_pose_bones)
            selected_bones.add(context.active_pose_bone.name)
        except (AttributeError, TypeError):
            return

        def is_selected(names):
            # Returns whether any of the named bones are selected.
            if isinstance(names, list) or isinstance(names, set):
                return not selected_bones.isdisjoint(names)
            elif names in selected_bones:
                return True
            return False

        num_rig_separators = [-1]

        def emit_rig_separator():
            if num_rig_separators[0] >= 0:
                layout.separator()
            num_rig_separators[0] += 1

        if is_selected({'upper_arm_fk.L', 'upper_arm_tweak.L.001', 'upper_arm_ik_target.L', 'hand_tweak.L', 'upper_arm_ik.L', 'forearm_tweak.L.001', 'VIS_upper_arm_ik_pole.L', 'hand_ik.L', 'forearm_fk.L', 'upper_arm_parent.L', 'upper_arm_tweak.L', 'hand_fk.L', 'forearm_tweak.L'}):
            emit_rig_separator()
            if is_selected({'upper_arm_parent.L', 'upper_arm_fk.L', 'hand_fk.L', 'forearm_fk.L'}):
                layout.prop(pose_bones['upper_arm_parent.L'], '["FK_limb_follow"]', text='FK Limb Follow', slider=True)
            layout.prop(pose_bones['upper_arm_parent.L'], '["IK_FK"]', text='IK-FK (hand.L)', slider=True)
            group1 = layout.column(align=True)
            props = group1.operator('pose.rigify_generic_snap_mw6ywfy2f2374456', text='FK->IK (hand.L)', icon='SNAP_ON')
            props.output_bones = '["upper_arm_fk.L", "forearm_fk.L", "hand_fk.L"]'
            props.input_bones = '["upper_arm_ik.L", "MCH-forearm_ik.L", "MCH-upper_arm_ik_target.L"]'
            props.ctrl_bones = '["upper_arm_ik.L", "upper_arm_ik_target.L", "hand_ik.L"]'
            group2 = group1.row(align=True)
            props = group2.operator('pose.rigify_generic_snap_bake_mw6ywfy2f2374456', text='Action', icon='ACTION_TWEAK')
            props.output_bones = '["upper_arm_fk.L", "forearm_fk.L", "hand_fk.L"]'
            props.input_bones = '["upper_arm_ik.L", "MCH-forearm_ik.L", "MCH-upper_arm_ik_target.L"]'
            props.ctrl_bones = '["upper_arm_ik.L", "upper_arm_ik_target.L", "hand_ik.L"]'
            props = group2.operator('pose.rigify_clear_keyframes_mw6ywfy2f2374456', text='Clear', icon='CANCEL')
            props.bones = '["upper_arm_fk.L", "forearm_fk.L", "hand_fk.L"]'
            group1 = layout.column(align=True)
            props = group1.operator('pose.rigify_limb_ik2fk_mw6ywfy2f2374456', text='IK->FK (hand.L)', icon='SNAP_ON')
            props.prop_bone = 'upper_arm_parent.L'
            props.fk_bones = '["upper_arm_fk.L", "forearm_fk.L", "hand_fk.L"]'
            props.ik_bones = '["upper_arm_ik.L", "MCH-forearm_ik.L", "MCH-upper_arm_ik_target.L"]'
            props.ctrl_bones = '["upper_arm_ik.L", "upper_arm_ik_target.L", "hand_ik.L"]'
            props.tail_bones = '[]'
            props.extra_ctrls = '[]'
            group2 = group1.row(align=True)
            props = group2.operator('pose.rigify_limb_ik2fk_bake_mw6ywfy2f2374456', text='Action', icon='ACTION_TWEAK')
            props.prop_bone = 'upper_arm_parent.L'
            props.fk_bones = '["upper_arm_fk.L", "forearm_fk.L", "hand_fk.L"]'
            props.ik_bones = '["upper_arm_ik.L", "MCH-forearm_ik.L", "MCH-upper_arm_ik_target.L"]'
            props.ctrl_bones = '["upper_arm_ik.L", "upper_arm_ik_target.L", "hand_ik.L"]'
            props.tail_bones = '[]'
            props.extra_ctrls = '[]'
            props = group2.operator('pose.rigify_clear_keyframes_mw6ywfy2f2374456', text='Clear', icon='CANCEL')
            props.bones = '["upper_arm_ik.L", "upper_arm_ik_target.L", "hand_ik.L"]'
            if is_selected({'upper_arm_parent.L', 'hand_ik.L', 'upper_arm_ik_target.L', 'upper_arm_ik.L'}):
                layout.prop(pose_bones['upper_arm_parent.L'], '["IK_Stretch"]', text='IK Stretch', slider=True)
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_limb_toggle_pole_mw6ywfy2f2374456', icon='FORCE_MAGNETIC')
                props.prop_bone = 'upper_arm_parent.L'
                props.ik_bones = '["upper_arm_ik.L", "MCH-forearm_ik.L", "MCH-upper_arm_ik_target.L"]'
                props.ctrl_bones = '["upper_arm_ik.L", "upper_arm_ik_target.L", "hand_ik.L"]'
                props.extra_ctrls = '[]'
                group2.prop(pose_bones['upper_arm_parent.L'], '["pole_vector"]', text='')
                props = group1.operator('pose.rigify_limb_toggle_pole_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.prop_bone = 'upper_arm_parent.L'
                props.ik_bones = '["upper_arm_ik.L", "MCH-forearm_ik.L", "MCH-upper_arm_ik_target.L"]'
                props.ctrl_bones = '["upper_arm_ik.L", "upper_arm_ik_target.L", "hand_ik.L"]'
                props.extra_ctrls = '[]'
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='IK Parent', icon='DOWNARROW_HLT')
                props.bone = 'hand_ik.L'
                props.prop_bone = 'upper_arm_parent.L'
                props.prop_id = 'IK_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "shoulder.L"]'
                props.locks = (False, False, False)
                group2.prop(pose_bones['upper_arm_parent.L'], '["IK_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'hand_ik.L'
                props.prop_bone = 'upper_arm_parent.L'
                props.prop_id = 'IK_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "shoulder.L"]'
                props.locks = (False, False, False)
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='Pole Parent', icon='DOWNARROW_HLT')
                props.bone = 'upper_arm_ik_target.L'
                props.prop_bone = 'upper_arm_parent.L'
                props.prop_id = 'pole_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "shoulder.L", "hand_ik.L"]'
                props.locks = (False, True, True)
                group2.prop(pose_bones['upper_arm_parent.L'], '["pole_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'upper_arm_ik_target.L'
                props.prop_bone = 'upper_arm_parent.L'
                props.prop_id = 'pole_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "shoulder.L", "hand_ik.L"]'
                props.locks = (False, True, True)
            if is_selected({'upper_arm_tweak.L.001'}):
                layout.prop(pose_bones['upper_arm_tweak.L.001'], '["rubber_tweak"]', text='Rubber Tweak (upper_arm.L)', slider=True)
            if is_selected({'forearm_tweak.L'}):
                layout.prop(pose_bones['forearm_tweak.L'], '["rubber_tweak"]', text='Rubber Tweak (forearm.L)', slider=True)
            if is_selected({'forearm_tweak.L.001'}):
                layout.prop(pose_bones['forearm_tweak.L.001'], '["rubber_tweak"]', text='Rubber Tweak (forearm.L)', slider=True)

        if is_selected({'f_index.02.L', 'f_index.01.L', 'f_index.01.L.001', 'f_index.03.L', 'f_index.01_master.L'}):
            emit_rig_separator()
            layout.prop(pose_bones['f_index.01_master.L'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'thumb.02.L', 'thumb.03.L', 'thumb.01.L', 'thumb.01_master.L', 'thumb.01.L.001'}):
            emit_rig_separator()
            layout.prop(pose_bones['thumb.01_master.L'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'f_middle.02.L', 'f_middle.01.L.001', 'f_middle.03.L', 'f_middle.01_master.L', 'f_middle.01.L'}):
            emit_rig_separator()
            layout.prop(pose_bones['f_middle.01_master.L'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'f_ring.01.L.001', 'f_ring.02.L', 'f_ring.01.L', 'f_ring.03.L', 'f_ring.01_master.L'}):
            emit_rig_separator()
            layout.prop(pose_bones['f_ring.01_master.L'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'f_pinky.01.L.001', 'f_pinky.03.L', 'f_pinky.02.L', 'f_pinky.01_master.L', 'f_pinky.01.L'}):
            emit_rig_separator()
            layout.prop(pose_bones['f_pinky.01_master.L'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'upper_arm_ik_target.R', 'hand_ik.R', 'upper_arm_tweak.R.001', 'hand_fk.R', 'upper_arm_tweak.R', 'upper_arm_parent.R', 'VIS_upper_arm_ik_pole.R', 'forearm_tweak.R', 'upper_arm_fk.R', 'forearm_tweak.R.001', 'hand_tweak.R', 'upper_arm_ik.R', 'forearm_fk.R'}):
            emit_rig_separator()
            if is_selected({'forearm_fk.R', 'hand_fk.R', 'upper_arm_fk.R', 'upper_arm_parent.R'}):
                layout.prop(pose_bones['upper_arm_parent.R'], '["FK_limb_follow"]', text='FK Limb Follow', slider=True)
            layout.prop(pose_bones['upper_arm_parent.R'], '["IK_FK"]', text='IK-FK (hand.R)', slider=True)
            group1 = layout.column(align=True)
            props = group1.operator('pose.rigify_generic_snap_mw6ywfy2f2374456', text='FK->IK (hand.R)', icon='SNAP_ON')
            props.output_bones = '["upper_arm_fk.R", "forearm_fk.R", "hand_fk.R"]'
            props.input_bones = '["upper_arm_ik.R", "MCH-forearm_ik.R", "MCH-upper_arm_ik_target.R"]'
            props.ctrl_bones = '["upper_arm_ik.R", "upper_arm_ik_target.R", "hand_ik.R"]'
            group2 = group1.row(align=True)
            props = group2.operator('pose.rigify_generic_snap_bake_mw6ywfy2f2374456', text='Action', icon='ACTION_TWEAK')
            props.output_bones = '["upper_arm_fk.R", "forearm_fk.R", "hand_fk.R"]'
            props.input_bones = '["upper_arm_ik.R", "MCH-forearm_ik.R", "MCH-upper_arm_ik_target.R"]'
            props.ctrl_bones = '["upper_arm_ik.R", "upper_arm_ik_target.R", "hand_ik.R"]'
            props = group2.operator('pose.rigify_clear_keyframes_mw6ywfy2f2374456', text='Clear', icon='CANCEL')
            props.bones = '["upper_arm_fk.R", "forearm_fk.R", "hand_fk.R"]'
            group1 = layout.column(align=True)
            props = group1.operator('pose.rigify_limb_ik2fk_mw6ywfy2f2374456', text='IK->FK (hand.R)', icon='SNAP_ON')
            props.prop_bone = 'upper_arm_parent.R'
            props.fk_bones = '["upper_arm_fk.R", "forearm_fk.R", "hand_fk.R"]'
            props.ik_bones = '["upper_arm_ik.R", "MCH-forearm_ik.R", "MCH-upper_arm_ik_target.R"]'
            props.ctrl_bones = '["upper_arm_ik.R", "upper_arm_ik_target.R", "hand_ik.R"]'
            props.tail_bones = '[]'
            props.extra_ctrls = '[]'
            group2 = group1.row(align=True)
            props = group2.operator('pose.rigify_limb_ik2fk_bake_mw6ywfy2f2374456', text='Action', icon='ACTION_TWEAK')
            props.prop_bone = 'upper_arm_parent.R'
            props.fk_bones = '["upper_arm_fk.R", "forearm_fk.R", "hand_fk.R"]'
            props.ik_bones = '["upper_arm_ik.R", "MCH-forearm_ik.R", "MCH-upper_arm_ik_target.R"]'
            props.ctrl_bones = '["upper_arm_ik.R", "upper_arm_ik_target.R", "hand_ik.R"]'
            props.tail_bones = '[]'
            props.extra_ctrls = '[]'
            props = group2.operator('pose.rigify_clear_keyframes_mw6ywfy2f2374456', text='Clear', icon='CANCEL')
            props.bones = '["upper_arm_ik.R", "upper_arm_ik_target.R", "hand_ik.R"]'
            if is_selected({'upper_arm_ik.R', 'hand_ik.R', 'upper_arm_ik_target.R', 'upper_arm_parent.R'}):
                layout.prop(pose_bones['upper_arm_parent.R'], '["IK_Stretch"]', text='IK Stretch', slider=True)
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_limb_toggle_pole_mw6ywfy2f2374456', icon='FORCE_MAGNETIC')
                props.prop_bone = 'upper_arm_parent.R'
                props.ik_bones = '["upper_arm_ik.R", "MCH-forearm_ik.R", "MCH-upper_arm_ik_target.R"]'
                props.ctrl_bones = '["upper_arm_ik.R", "upper_arm_ik_target.R", "hand_ik.R"]'
                props.extra_ctrls = '[]'
                group2.prop(pose_bones['upper_arm_parent.R'], '["pole_vector"]', text='')
                props = group1.operator('pose.rigify_limb_toggle_pole_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.prop_bone = 'upper_arm_parent.R'
                props.ik_bones = '["upper_arm_ik.R", "MCH-forearm_ik.R", "MCH-upper_arm_ik_target.R"]'
                props.ctrl_bones = '["upper_arm_ik.R", "upper_arm_ik_target.R", "hand_ik.R"]'
                props.extra_ctrls = '[]'
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='IK Parent', icon='DOWNARROW_HLT')
                props.bone = 'hand_ik.R'
                props.prop_bone = 'upper_arm_parent.R'
                props.prop_id = 'IK_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "shoulder.R"]'
                props.locks = (False, False, False)
                group2.prop(pose_bones['upper_arm_parent.R'], '["IK_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'hand_ik.R'
                props.prop_bone = 'upper_arm_parent.R'
                props.prop_id = 'IK_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "shoulder.R"]'
                props.locks = (False, False, False)
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='Pole Parent', icon='DOWNARROW_HLT')
                props.bone = 'upper_arm_ik_target.R'
                props.prop_bone = 'upper_arm_parent.R'
                props.prop_id = 'pole_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "shoulder.R", "hand_ik.R"]'
                props.locks = (False, True, True)
                group2.prop(pose_bones['upper_arm_parent.R'], '["pole_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'upper_arm_ik_target.R'
                props.prop_bone = 'upper_arm_parent.R'
                props.prop_id = 'pole_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "shoulder.R", "hand_ik.R"]'
                props.locks = (False, True, True)
            if is_selected({'upper_arm_tweak.R.001'}):
                layout.prop(pose_bones['upper_arm_tweak.R.001'], '["rubber_tweak"]', text='Rubber Tweak (upper_arm.R)', slider=True)
            if is_selected({'forearm_tweak.R'}):
                layout.prop(pose_bones['forearm_tweak.R'], '["rubber_tweak"]', text='Rubber Tweak (forearm.R)', slider=True)
            if is_selected({'forearm_tweak.R.001'}):
                layout.prop(pose_bones['forearm_tweak.R.001'], '["rubber_tweak"]', text='Rubber Tweak (forearm.R)', slider=True)

        if is_selected({'f_index.02.R', 'f_index.01.R.001', 'f_index.01.R', 'f_index.01_master.R', 'f_index.03.R'}):
            emit_rig_separator()
            layout.prop(pose_bones['f_index.01_master.R'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'thumb.01.R', 'thumb.02.R', 'thumb.01_master.R', 'thumb.01.R.001', 'thumb.03.R'}):
            emit_rig_separator()
            layout.prop(pose_bones['thumb.01_master.R'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'f_middle.01_master.R', 'f_middle.02.R', 'f_middle.01.R', 'f_middle.01.R.001', 'f_middle.03.R'}):
            emit_rig_separator()
            layout.prop(pose_bones['f_middle.01_master.R'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'f_ring.02.R', 'f_ring.01_master.R', 'f_ring.01.R', 'f_ring.01.R.001', 'f_ring.03.R'}):
            emit_rig_separator()
            layout.prop(pose_bones['f_ring.01_master.R'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'f_pinky.01_master.R', 'f_pinky.03.R', 'f_pinky.02.R', 'f_pinky.01.R.001', 'f_pinky.01.R'}):
            emit_rig_separator()
            layout.prop(pose_bones['f_pinky.01_master.R'], '["finger_curve"]', text='Curvature', slider=True)

        if is_selected({'neck', 'tweak_spine.003', 'hips', 'head', 'tweak_spine', 'spine_fk', 'tweak_spine.002', 'spine_fk.004', 'chest', 'spine_fk.002', 'tweak_spine.005', 'tweak_spine.001', 'torso', 'spine_fk.001', 'tweak_spine.004', 'spine_fk.003'}):
            emit_rig_separator()
            layout.prop(pose_bones['torso'], '["neck_follow"]', text='Neck Follow', slider=True)
            layout.prop(pose_bones['torso'], '["head_follow"]', text='Head Follow', slider=True)
            if is_selected({'spine_fk.004', 'tweak_spine.003', 'chest', 'spine_fk.002', 'tweak_spine.001', 'tweak_spine.005', 'tweak_spine', 'torso', 'hips', 'spine_fk', 'spine_fk.001', 'tweak_spine.004', 'spine_fk.003', 'tweak_spine.002'}):
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='Torso Parent', icon='DOWNARROW_HLT')
                props.bone = 'torso'
                props.prop_bone = 'torso'
                props.prop_id = 'torso_parent'
                props.parent_names = '["None", "Root"]'
                props.locks = (False, False, False)
                group2.prop(pose_bones['torso'], '["torso_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'torso'
                props.prop_bone = 'torso'
                props.prop_id = 'torso_parent'
                props.parent_names = '["None", "Root"]'
                props.locks = (False, False, False)

        if is_selected({'lid.B.L.001', 'eye.L', 'lid.B.L.002', 'eye_master.L', 'lid.B.L.003', 'lid.T.L', 'eye.R', 'lid.B.L', 'lid.T.L.002', 'lid.T.L.003', 'lid.T.L.001', 'eye_common', 'eye_master.R'}):
            emit_rig_separator()
            if is_selected({'eye_common'}):
                layout.prop(pose_bones['eye.L'], '["lid_follow"]', text='Eyelids Follow (eye.L)', slider=True)
                layout.prop(pose_bones['eye.R'], '["lid_follow"]', text='Eyelids Follow (eye.R)', slider=True)
            if is_selected({'lid.B.L.001', 'eye.L', 'lid.B.L.002', 'eye_master.L', 'lid.B.L.003', 'lid.T.L', 'lid.B.L', 'lid.T.L.002', 'lid.T.L.003', 'lid.T.L.001'}):
                layout.prop(pose_bones['eye.L'], '["lid_follow"]', text='Eyelids Follow', slider=True)
            if is_selected({'eye.L', 'eye_master.L', 'eye_common', 'eye.R', 'eye_master.R'}):
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='Parent (eye_common)', icon='DOWNARROW_HLT')
                props.bone = 'eye_common'
                props.prop_bone = 'eye_common'
                props.prop_id = 'parent_switch'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "face"]'
                props.locks = (False, False, False)
                group2.prop(pose_bones['eye_common'], '["parent_switch"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'eye_common'
                props.prop_bone = 'eye_common'
                props.prop_id = 'parent_switch'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "face"]'
                props.locks = (False, False, False)

        if is_selected({'lid.T.R.003', 'lid.T.R', 'lid.B.R.001', 'eye.R', 'lid.B.R.002', 'lid.B.R', 'lid.B.R.003', 'lid.T.R.002', 'lid.T.R.001', 'eye_master.R'}):
            emit_rig_separator()
            layout.prop(pose_bones['eye.R'], '["lid_follow"]', text='Eyelids Follow', slider=True)

        if is_selected({'jaw_master', 'jaw_master_mouth'}):
            emit_rig_separator()
            layout.prop(pose_bones['jaw_master'], '["mouth_lock"]', text='Mouth Lock', slider=True)

        if is_selected({'thigh_tweak.L.001', 'thigh_ik_target.L', 'foot_tweak.L', 'VIS_thigh_ik_pole.L', 'thigh_fk.L', 'toe_fk.L', 'shin_tweak.L.001', 'foot_spin_ik.L', 'foot_fk.L', 'foot_heel_ik.L', 'foot_ik.L', 'shin_fk.L', 'thigh_parent.L', 'thigh_ik.L', 'shin_tweak.L', 'toe_ik.L', 'thigh_tweak.L'}):
            emit_rig_separator()
            if is_selected({'thigh_fk.L', 'toe_fk.L', 'shin_fk.L', 'foot_fk.L', 'thigh_parent.L'}):
                layout.prop(pose_bones['thigh_parent.L'], '["FK_limb_follow"]', text='FK Limb Follow', slider=True)
            layout.prop(pose_bones['thigh_parent.L'], '["IK_FK"]', text='IK-FK (foot.L)', slider=True)
            group1 = layout.column(align=True)
            props = group1.operator('pose.rigify_generic_snap_mw6ywfy2f2374456', text='FK->IK (foot.L)', icon='SNAP_ON')
            props.output_bones = '["thigh_fk.L", "shin_fk.L", "foot_fk.L", "toe_fk.L"]'
            props.input_bones = '["thigh_ik.L", "MCH-shin_ik.L", "MCH-thigh_ik_target.L", "toe_ik.L"]'
            props.ctrl_bones = '["thigh_ik.L", "thigh_ik_target.L", "foot_ik.L", "toe_ik.L", "foot_heel_ik.L", "foot_spin_ik.L"]'
            group2 = group1.row(align=True)
            props = group2.operator('pose.rigify_generic_snap_bake_mw6ywfy2f2374456', text='Action', icon='ACTION_TWEAK')
            props.output_bones = '["thigh_fk.L", "shin_fk.L", "foot_fk.L", "toe_fk.L"]'
            props.input_bones = '["thigh_ik.L", "MCH-shin_ik.L", "MCH-thigh_ik_target.L", "toe_ik.L"]'
            props.ctrl_bones = '["thigh_ik.L", "thigh_ik_target.L", "foot_ik.L", "toe_ik.L", "foot_heel_ik.L", "foot_spin_ik.L"]'
            props = group2.operator('pose.rigify_clear_keyframes_mw6ywfy2f2374456', text='Clear', icon='CANCEL')
            props.bones = '["thigh_fk.L", "shin_fk.L", "foot_fk.L", "toe_fk.L"]'
            group1 = layout.column(align=True)
            props = group1.operator('pose.rigify_limb_ik2fk_mw6ywfy2f2374456', text='IK->FK (foot.L)', icon='SNAP_ON')
            props.prop_bone = 'thigh_parent.L'
            props.fk_bones = '["thigh_fk.L", "shin_fk.L", "foot_fk.L", "toe_fk.L"]'
            props.ik_bones = '["thigh_ik.L", "MCH-shin_ik.L", "MCH-thigh_ik_target.L"]'
            props.ctrl_bones = '["thigh_ik.L", "thigh_ik_target.L", "foot_ik.L"]'
            props.tail_bones = '["toe_ik.L"]'
            props.extra_ctrls = '["foot_heel_ik.L", "foot_spin_ik.L"]'
            group2 = group1.row(align=True)
            props = group2.operator('pose.rigify_limb_ik2fk_bake_mw6ywfy2f2374456', text='Action', icon='ACTION_TWEAK')
            props.prop_bone = 'thigh_parent.L'
            props.fk_bones = '["thigh_fk.L", "shin_fk.L", "foot_fk.L", "toe_fk.L"]'
            props.ik_bones = '["thigh_ik.L", "MCH-shin_ik.L", "MCH-thigh_ik_target.L"]'
            props.ctrl_bones = '["thigh_ik.L", "thigh_ik_target.L", "foot_ik.L"]'
            props.tail_bones = '["toe_ik.L"]'
            props.extra_ctrls = '["foot_heel_ik.L", "foot_spin_ik.L"]'
            props = group2.operator('pose.rigify_clear_keyframes_mw6ywfy2f2374456', text='Clear', icon='CANCEL')
            props.bones = '["thigh_ik.L", "thigh_ik_target.L", "foot_ik.L", "toe_ik.L", "foot_heel_ik.L", "foot_spin_ik.L"]'
            if is_selected({'foot_heel_ik.L', 'foot_ik.L', 'thigh_parent.L', 'thigh_ik_target.L', 'toe_ik.L', 'foot_spin_ik.L', 'thigh_ik.L'}):
                layout.prop(pose_bones['thigh_parent.L'], '["IK_Stretch"]', text='IK Stretch', slider=True)
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_limb_toggle_pole_mw6ywfy2f2374456', icon='FORCE_MAGNETIC')
                props.prop_bone = 'thigh_parent.L'
                props.ik_bones = '["thigh_ik.L", "MCH-shin_ik.L", "MCH-thigh_ik_target.L"]'
                props.ctrl_bones = '["thigh_ik.L", "thigh_ik_target.L", "foot_ik.L"]'
                props.extra_ctrls = '["foot_heel_ik.L", "foot_spin_ik.L"]'
                group2.prop(pose_bones['thigh_parent.L'], '["pole_vector"]', text='')
                props = group1.operator('pose.rigify_limb_toggle_pole_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.prop_bone = 'thigh_parent.L'
                props.ik_bones = '["thigh_ik.L", "MCH-shin_ik.L", "MCH-thigh_ik_target.L"]'
                props.ctrl_bones = '["thigh_ik.L", "thigh_ik_target.L", "foot_ik.L"]'
                props.extra_ctrls = '["foot_heel_ik.L", "foot_spin_ik.L"]'
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='IK Parent', icon='DOWNARROW_HLT')
                props.bone = 'foot_ik.L'
                props.prop_bone = 'thigh_parent.L'
                props.prop_id = 'IK_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head"]'
                props.locks = (False, False, False)
                group2.prop(pose_bones['thigh_parent.L'], '["IK_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'foot_ik.L'
                props.prop_bone = 'thigh_parent.L'
                props.prop_id = 'IK_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head"]'
                props.locks = (False, False, False)
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='Pole Parent', icon='DOWNARROW_HLT')
                props.bone = 'thigh_ik_target.L'
                props.prop_bone = 'thigh_parent.L'
                props.prop_id = 'pole_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "foot_ik.L"]'
                props.locks = (False, True, True)
                group2.prop(pose_bones['thigh_parent.L'], '["pole_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'thigh_ik_target.L'
                props.prop_bone = 'thigh_parent.L'
                props.prop_id = 'pole_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "foot_ik.L"]'
                props.locks = (False, True, True)
            if is_selected({'thigh_tweak.L.001'}):
                layout.prop(pose_bones['thigh_tweak.L.001'], '["rubber_tweak"]', text='Rubber Tweak (thigh.L)', slider=True)
            if is_selected({'shin_tweak.L'}):
                layout.prop(pose_bones['shin_tweak.L'], '["rubber_tweak"]', text='Rubber Tweak (shin.L)', slider=True)
            if is_selected({'shin_tweak.L.001'}):
                layout.prop(pose_bones['shin_tweak.L.001'], '["rubber_tweak"]', text='Rubber Tweak (shin.L)', slider=True)

        if is_selected({'foot_fk.R', 'thigh_tweak.R.001', 'thigh_ik.R', 'shin_fk.R', 'foot_ik.R', 'foot_tweak.R', 'thigh_tweak.R', 'VIS_thigh_ik_pole.R', 'thigh_fk.R', 'foot_heel_ik.R', 'toe_ik.R', 'shin_tweak.R', 'shin_tweak.R.001', 'thigh_ik_target.R', 'thigh_parent.R', 'foot_spin_ik.R', 'toe_fk.R'}):
            emit_rig_separator()
            if is_selected({'thigh_parent.R', 'foot_fk.R', 'toe_fk.R', 'thigh_fk.R', 'shin_fk.R'}):
                layout.prop(pose_bones['thigh_parent.R'], '["FK_limb_follow"]', text='FK Limb Follow', slider=True)
            layout.prop(pose_bones['thigh_parent.R'], '["IK_FK"]', text='IK-FK (foot.R)', slider=True)
            group1 = layout.column(align=True)
            props = group1.operator('pose.rigify_generic_snap_mw6ywfy2f2374456', text='FK->IK (foot.R)', icon='SNAP_ON')
            props.output_bones = '["thigh_fk.R", "shin_fk.R", "foot_fk.R", "toe_fk.R"]'
            props.input_bones = '["thigh_ik.R", "MCH-shin_ik.R", "MCH-thigh_ik_target.R", "toe_ik.R"]'
            props.ctrl_bones = '["thigh_ik.R", "thigh_ik_target.R", "foot_ik.R", "toe_ik.R", "foot_heel_ik.R", "foot_spin_ik.R"]'
            group2 = group1.row(align=True)
            props = group2.operator('pose.rigify_generic_snap_bake_mw6ywfy2f2374456', text='Action', icon='ACTION_TWEAK')
            props.output_bones = '["thigh_fk.R", "shin_fk.R", "foot_fk.R", "toe_fk.R"]'
            props.input_bones = '["thigh_ik.R", "MCH-shin_ik.R", "MCH-thigh_ik_target.R", "toe_ik.R"]'
            props.ctrl_bones = '["thigh_ik.R", "thigh_ik_target.R", "foot_ik.R", "toe_ik.R", "foot_heel_ik.R", "foot_spin_ik.R"]'
            props = group2.operator('pose.rigify_clear_keyframes_mw6ywfy2f2374456', text='Clear', icon='CANCEL')
            props.bones = '["thigh_fk.R", "shin_fk.R", "foot_fk.R", "toe_fk.R"]'
            group1 = layout.column(align=True)
            props = group1.operator('pose.rigify_limb_ik2fk_mw6ywfy2f2374456', text='IK->FK (foot.R)', icon='SNAP_ON')
            props.prop_bone = 'thigh_parent.R'
            props.fk_bones = '["thigh_fk.R", "shin_fk.R", "foot_fk.R", "toe_fk.R"]'
            props.ik_bones = '["thigh_ik.R", "MCH-shin_ik.R", "MCH-thigh_ik_target.R"]'
            props.ctrl_bones = '["thigh_ik.R", "thigh_ik_target.R", "foot_ik.R"]'
            props.tail_bones = '["toe_ik.R"]'
            props.extra_ctrls = '["foot_heel_ik.R", "foot_spin_ik.R"]'
            group2 = group1.row(align=True)
            props = group2.operator('pose.rigify_limb_ik2fk_bake_mw6ywfy2f2374456', text='Action', icon='ACTION_TWEAK')
            props.prop_bone = 'thigh_parent.R'
            props.fk_bones = '["thigh_fk.R", "shin_fk.R", "foot_fk.R", "toe_fk.R"]'
            props.ik_bones = '["thigh_ik.R", "MCH-shin_ik.R", "MCH-thigh_ik_target.R"]'
            props.ctrl_bones = '["thigh_ik.R", "thigh_ik_target.R", "foot_ik.R"]'
            props.tail_bones = '["toe_ik.R"]'
            props.extra_ctrls = '["foot_heel_ik.R", "foot_spin_ik.R"]'
            props = group2.operator('pose.rigify_clear_keyframes_mw6ywfy2f2374456', text='Clear', icon='CANCEL')
            props.bones = '["thigh_ik.R", "thigh_ik_target.R", "foot_ik.R", "toe_ik.R", "foot_heel_ik.R", "foot_spin_ik.R"]'
            if is_selected({'foot_heel_ik.R', 'toe_ik.R', 'thigh_ik_target.R', 'foot_ik.R', 'thigh_parent.R', 'foot_spin_ik.R', 'thigh_ik.R'}):
                layout.prop(pose_bones['thigh_parent.R'], '["IK_Stretch"]', text='IK Stretch', slider=True)
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_limb_toggle_pole_mw6ywfy2f2374456', icon='FORCE_MAGNETIC')
                props.prop_bone = 'thigh_parent.R'
                props.ik_bones = '["thigh_ik.R", "MCH-shin_ik.R", "MCH-thigh_ik_target.R"]'
                props.ctrl_bones = '["thigh_ik.R", "thigh_ik_target.R", "foot_ik.R"]'
                props.extra_ctrls = '["foot_heel_ik.R", "foot_spin_ik.R"]'
                group2.prop(pose_bones['thigh_parent.R'], '["pole_vector"]', text='')
                props = group1.operator('pose.rigify_limb_toggle_pole_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.prop_bone = 'thigh_parent.R'
                props.ik_bones = '["thigh_ik.R", "MCH-shin_ik.R", "MCH-thigh_ik_target.R"]'
                props.ctrl_bones = '["thigh_ik.R", "thigh_ik_target.R", "foot_ik.R"]'
                props.extra_ctrls = '["foot_heel_ik.R", "foot_spin_ik.R"]'
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='IK Parent', icon='DOWNARROW_HLT')
                props.bone = 'foot_ik.R'
                props.prop_bone = 'thigh_parent.R'
                props.prop_id = 'IK_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head"]'
                props.locks = (False, False, False)
                group2.prop(pose_bones['thigh_parent.R'], '["IK_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'foot_ik.R'
                props.prop_bone = 'thigh_parent.R'
                props.prop_id = 'IK_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head"]'
                props.locks = (False, False, False)
                group1 = layout.row(align=True)
                group2 = group1.split(factor=0.75, align=True)
                props = group2.operator('pose.rigify_switch_parent_mw6ywfy2f2374456', text='Pole Parent', icon='DOWNARROW_HLT')
                props.bone = 'thigh_ik_target.R'
                props.prop_bone = 'thigh_parent.R'
                props.prop_id = 'pole_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "foot_ik.R"]'
                props.locks = (False, True, True)
                group2.prop(pose_bones['thigh_parent.R'], '["pole_parent"]', text='')
                props = group1.operator('pose.rigify_switch_parent_bake_mw6ywfy2f2374456', text='', icon='ACTION_TWEAK')
                props.bone = 'thigh_ik_target.R'
                props.prop_bone = 'thigh_parent.R'
                props.prop_id = 'pole_parent'
                props.parent_names = '["None", "Root", "Torso", "Hips", "Chest", "Head", "foot_ik.R"]'
                props.locks = (False, True, True)
            if is_selected({'thigh_tweak.R.001'}):
                layout.prop(pose_bones['thigh_tweak.R.001'], '["rubber_tweak"]', text='Rubber Tweak (thigh.R)', slider=True)
            if is_selected({'shin_tweak.R'}):
                layout.prop(pose_bones['shin_tweak.R'], '["rubber_tweak"]', text='Rubber Tweak (shin.R)', slider=True)
            if is_selected({'shin_tweak.R.001'}):
                layout.prop(pose_bones['shin_tweak.R.001'], '["rubber_tweak"]', text='Rubber Tweak (shin.R)', slider=True)

class RigBakeSettings(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = "Rig Bake Settings"
    bl_idname = "VIEW3D_PT_rig_bake_settings_" + rig_id
    bl_category = 'Item'

    @classmethod
    def poll(self, context):
        return RigUI.poll(context) and find_action(context.active_object) is not None

    def draw(self, context):
        RigifyBakeKeyframesMixin.draw_common_bake_ui(context, self.layout)

class RigLayers(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = "Rig Layers"
    bl_idname = "VIEW3D_PT_rig_layers_" + rig_id
    bl_category = 'Item'

    @classmethod
    def poll(self, context):
        try:
            return (context.active_object.data.get("rig_id") == rig_id)
        except (AttributeError, KeyError, TypeError):
            return False

    def draw(self, context):
        layout = self.layout
        col = layout.column()

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=0, toggle=True, text='Face')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=1, toggle=True, text='Face (Primary)')
        row.prop(context.active_object.data, 'layers', index=2, toggle=True, text='Face (Secondary)')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=3, toggle=True, text='Torso')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=4, toggle=True, text='Torso (Tweak)')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=5, toggle=True, text='Fingers')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=6, toggle=True, text='Fingers (Detail)')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=7, toggle=True, text='Arm.L (IK)')
        row.prop(context.active_object.data, 'layers', index=10, toggle=True, text='Arm.R (IK)')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=8, toggle=True, text='Arm.L (FK)')
        row.prop(context.active_object.data, 'layers', index=11, toggle=True, text='Arm.R (FK)')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=9, toggle=True, text='Arm.L (Tweak)')
        row.prop(context.active_object.data, 'layers', index=12, toggle=True, text='Arm.R (Tweak)')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=13, toggle=True, text='Leg.L (IK)')
        row.prop(context.active_object.data, 'layers', index=16, toggle=True, text='Leg.R (IK)')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=14, toggle=True, text='Leg.L (FK)')
        row.prop(context.active_object.data, 'layers', index=17, toggle=True, text='Leg.R (FK)')

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=15, toggle=True, text='Leg.L (Tweak)')
        row.prop(context.active_object.data, 'layers', index=18, toggle=True, text='Leg.R (Tweak)')

        row = col.row()
        row.separator()
        row = col.row()
        row.separator()

        row = col.row()
        row.prop(context.active_object.data, 'layers', index=28, toggle=True, text='Root')

def register():
    bpy.utils.register_class(RigBakeSettings)
    bpy.utils.register_class(RigUI)
    bpy.utils.register_class(RigLayers)
    bpy.utils.register_class(RIGIFY_OT_get_frame_range)
    bpy.utils.register_class(POSE_OT_rigify_generic_snap)
    bpy.utils.register_class(POSE_OT_rigify_generic_snap_bake)
    bpy.utils.register_class(POSE_OT_rigify_clear_keyframes)
    bpy.utils.register_class(POSE_OT_rigify_limb_ik2fk)
    bpy.utils.register_class(POSE_OT_rigify_limb_ik2fk_bake)
    bpy.utils.register_class(POSE_OT_rigify_limb_toggle_pole)
    bpy.utils.register_class(POSE_OT_rigify_limb_toggle_pole_bake)
    bpy.utils.register_class(POSE_OT_rigify_switch_parent)
    bpy.utils.register_class(POSE_OT_rigify_switch_parent_bake)

def unregister():
    bpy.utils.unregister_class(RigBakeSettings)
    bpy.utils.unregister_class(RigUI)
    bpy.utils.unregister_class(RigLayers)
    bpy.utils.unregister_class(RIGIFY_OT_get_frame_range)
    bpy.utils.unregister_class(POSE_OT_rigify_generic_snap)
    bpy.utils.unregister_class(POSE_OT_rigify_generic_snap_bake)
    bpy.utils.unregister_class(POSE_OT_rigify_clear_keyframes)
    bpy.utils.unregister_class(POSE_OT_rigify_limb_ik2fk)
    bpy.utils.unregister_class(POSE_OT_rigify_limb_ik2fk_bake)
    bpy.utils.unregister_class(POSE_OT_rigify_limb_toggle_pole)
    bpy.utils.unregister_class(POSE_OT_rigify_limb_toggle_pole_bake)
    bpy.utils.unregister_class(POSE_OT_rigify_switch_parent)
    bpy.utils.unregister_class(POSE_OT_rigify_switch_parent_bake)

register()
