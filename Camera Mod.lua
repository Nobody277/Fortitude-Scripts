local oc = {}
local sc = {}
local slc = {}
local rm = false
local ca = false
local cam = nil
local eps = 0

local fi = false
local fit = 1000
local fo = false
local fot = 1000
local fov = 50.0
local fd = 50.0
local cb = false

local si = 0
local lx = 0.0
local ly = 0.0
local lz = 0.0
local iv = 1.0
local fp = false
local ms = false
local ms_val = 10.0

function gi(v)
    local n = #oc
    if n == 0 then return false end
    local s = 1 + eps
    local tt = (n - 1) * s + 1
    if v == tt and tt == #sc then return true end
    if v > #sc then return false end
    local seg = (v - 1) // s + 1
    local off = (v - 1) % s
    return off == 0
end

function rc()
    if #oc < 2 then
        sc = {}
        for i, v in ipairs(oc) do
            table.insert(sc, v)
        end
        return
    end
    local nc = {}
    for i = 1, #oc - 1 do
        local sp = oc[i]
        local ep = oc[i + 1]
        table.insert(nc, {x = sp.x, y = sp.y, z = sp.z})
        if eps > 0 then
            for st = 1, eps do
                local a = st / (eps + 1)
                local dx, dy, dz = math.sub(ep.x, ep.y, ep.z, sp.x, sp.y, sp.z)
                local sx, sy, sz = math.scale(dx, dy, dz, a)
                local ix, iy, iz = math.add(sp.x, sp.y, sp.z, sx, sy, sz)
                table.insert(nc, {x = ix, y = iy, z = iz})
            end
        end
    end
    table.insert(nc, oc[#oc])
    sc = nc
end

function lp(x1, y1, z1, x2, y2, z2, a)
    local dx, dy, dz = math.sub(x2, y2, z2, x1, y1, z1)
    local sx, sy, sz = math.scale(dx, dy, dz, a)
    local fx, fy, fz = math.add(x1, y1, z1, sx, sy, sz)
    return fx, fy, fz
end

function gf(v)
    local ii = 0
    local rr = v
    while rr >= 1.0 do
        rr = rr - 1.0
        ii = ii + 1
    end
    return ii, rr
end

function gl(f)
    local L = #slc
    if L == 0 then
        return 0.0, 0.0, 0.0
    elseif L == 1 then
        return slc[1].x, slc[1].y, slc[1].z
    else
        local lm = L - 1
        local vs = f * lm
        local I, R = gf(vs)
        local si = I + 1
        if si >= L then
            return slc[L].x, slc[L].y, slc[L].z
        else
            local p1 = slc[si]
            local p2 = slc[si + 1]
            return lp(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, R)
        end
    end
end

-- https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline
local function catrom(p0, p1, p2, p3, t)
    local t2 = t * t
    local t3 = t2 * t

    local x = 0.5 * (
        (2 * p1.x) +
        (-p0.x + p2.x) * t +
        (2 * p0.x - 5 * p1.x + 4 * p2.x - p3.x) * t2 +
        (-p0.x + 3 * p1.x - 3 * p2.x + p3.x) * t3
    )
    local y = 0.5 * (
        (2 * p1.y) +
        (-p0.y + p2.y) * t +
        (2 * p0.y - 5 * p1.y + 4 * p2.y - p3.y) * t2 +
        (-p0.y + 3 * p1.y - 3 * p2.y + p3.y) * t3
    )
    local z = 0.5 * (
        (2 * p1.z) +
        (-p0.z + p2.z) * t +
        (2 * p0.z - 5 * p1.z + 4 * p2.z - p3.z) * t2 +
        (-p0.z + 3 * p1.z - 3 * p2.z + p3.z) * t3
    )
    return x, y, z
end

function ic(c, sp, ep, slp, elp, dur, idx)
    local dist = math.getDistance(sp.x, sp.y, sp.z, ep.x, ep.y, ep.z)
    local segment_time = dur
    if ms then
        segment_time = (dist / ms_val) * 1000
    end

    local p0 = sc[idx - 1] or sc[idx]
    local p1 = sc[idx]
    local p2 = sc[idx + 1]
    local p3 = sc[idx + 2] or sc[idx + 1]

    local l0, l1, l2, l3
    if #slc > 0 then
        l0 = slc[idx - 1] or slc[idx] or slc[1]
        l1 = slc[idx] or slc[#slc]
        l2 = slc[idx + 1] or l1
        l3 = slc[idx + 2] or l2
    else
        l0 = {x = 0.0, y = 0.0, z = 0.0}
        l1 = {x = 0.0, y = 0.0, z = 0.0}
        l2 = {x = 0.0, y = 0.0, z = 0.0}
        l3 = {x = 0.0, y = 0.0, z = 0.0}
    end

    local start_time = system.getTickCount64()
    local end_time = start_time + segment_time

    while ca do
        local now = system.getTickCount64()
        if now >= end_time then
            break
        end

        local alpha = (now - start_time) / segment_time
        if alpha > 1.0 then alpha = 1.0 end

        local cx, cy, cz = catrom(p0, p1, p2, p3, alpha)

        local lx_val, ly_val, lz_val = catrom(l0, l1, l2, l3, alpha)

        local dx, dy, dz = math.sub(lx_val, ly_val, lz_val, cx, cy, cz)
        local hd = 0.0
        local pt = 0.0

        if #slc > 0 and not fp then
            hd = natives.misc_getHeadingFromVector2d(dx, dy)
            local dd = math.getDistance(0.0, 0.0, 0.0, dx, dy, 0.0)
            if dd ~= 0.0 then
                local ratio = dz / dd
                local rad = math.atan(ratio)
                local deg = math.radiansToDegrees(rad)
                pt = deg
            end
        else
            hd = 0.0
            pt = 0.0
        end

        if fp then
            local px, py, pz = player.getLocalPedCoords()
            local dxp, dyp, dzp = math.sub(px, py, pz, cx, cy, cz)
            hd = natives.misc_getHeadingFromVector2d(dxp, dyp)
            local ddp = math.getDistance(0.0, 0.0, 0.0, dxp, dyp, 0.0)
            if ddp ~= 0.0 then
                local rp = dzp / ddp
                local rdp = math.atan(rp)
                local dgp = math.radiansToDegrees(rdp)
                pt = dgp
            end
        end

        natives.cam_setCamCoord(c, cx, cy, cz)
        natives.cam_setCamRot(c, pt, 0.0, hd, 2)
        system.yield(0)
    end

    natives.cam_setCamCoord(c, ep.x, ep.y, ep.z)

    local dx_f, dy_f, dz_f = math.sub(elp.x, elp.y, elp.z, ep.x, ep.y, ep.z)
    local final_hd = 0.0
    local final_pt = 0.0

    if #slc > 0 and not fp then
        final_hd = natives.misc_getHeadingFromVector2d(dx_f, dy_f)
        local final_hdf = math.getDistance(0.0, 0.0, 0.0, dx_f, dy_f, 0.0)
        if final_hdf ~= 0.0 then
            local ratio_f = dz_f / final_hdf
            local rad_f = math.atan(ratio_f)
            local deg_f = math.radiansToDegrees(rad_f)
            final_pt = deg_f
        end
    else
        final_hd = 0.0
        final_pt = 0.0
    end

    if fp then
        local px, py, pz = player.getLocalPedCoords()
        local dxp_f, dyp_f, dzp_f = math.sub(px, py, pz, ep.x, ep.y, ep.z)
        final_hd = natives.misc_getHeadingFromVector2d(dxp_f, dyp_f)
        local final_hdfp = math.getDistance(0.0, 0.0, 0.0, dxp_f, dyp_f, 0.0)
        if final_hdfp ~= 0.0 then
            local rrpf = dzp_f / final_hdfp
            local rdf = math.atan(rrpf)
            local dgrf = math.radiansToDegrees(rdf)
            final_pt = dgrf
        end
    end

    natives.cam_setCamRot(c, final_pt, 0.0, final_hd, 2)
end

function move_camera()
    if not cam then
        cam = natives.cam_createCam('DEFAULT_SCRIPTED_CAMERA', true)
    end
    if cb then
        natives.cam_requestLetterBoxNow(true, true)
    else
        natives.cam_requestLetterBoxNow(false, false)
    end
    natives.cam_setCamFov(cam, fov)
    natives.cam_setCamFocusDistance(cam, fd)

    local first_pos = sc[1]
    natives.cam_setCamCoord(cam, first_pos.x, first_pos.y, first_pos.z)

    local init_hd = 0.0
    local init_pt = 0.0
    if #slc > 0 and not fp then
        local fx, fy, fz = gl(0.0)
        local dx, dy, dz = math.sub(fx, fy, fz, first_pos.x, first_pos.y, first_pos.z)
        init_hd = natives.misc_getHeadingFromVector2d(dx, dy)
        local dist = math.getDistance(0.0, 0.0, 0.0, dx, dy, 0.0)
        if dist ~= 0.0 then
            local ratio = dz / dist
            local rd = math.atan(ratio)
            local deg = math.radiansToDegrees(rd)
            init_pt = deg
        end
    end

    if fp then
        local px, py, pz = player.getLocalPedCoords()
        local dxp, dyp, dzp = math.sub(px, py, pz, first_pos.x, first_pos.y, first_pos.z)
        init_hd = natives.misc_getHeadingFromVector2d(dxp, dyp)
        local distp = math.getDistance(0.0, 0.0, 0.0, dxp, dyp, 0.0)
        if distp ~= 0.0 then
            local rp = dzp / distp
            local rdp = math.atan(rp)
            local dgp = math.radiansToDegrees(rdp)
            init_pt = dgp
        end
    end

    natives.cam_setCamRot(cam, init_pt, 0.0, init_hd, 2)
    natives.cam_renderScriptCams(true, false, 0, true, false, 0)

    if fi then
        natives.cam_doScreenFadeOut(0)
        system.yield(100)
        natives.cam_doScreenFadeIn(fit)
    end

    local total_segments = #sc - 1
    local M = #sc
    local L = #slc

    for i = 1, total_segments do
        if not ca then break end
        local sp = sc[i]
        local ep = sc[i + 1]

        local sf = 0.0
        local ef = 0.0
        if M > 1 then
            sf = (i - 1) / (M - 1)
            ef = i / (M - 1)
        end

        local slx, sly, slz
        local elx, ely, elz
        if L == 0 then
            slx, sly, slz = sp.x, sp.y, sp.z
            elx, ely, elz = ep.x, ep.y, ep.z
        else
            slx, sly, slz = gl(sf)
            elx, ely, elz = gl(ef)
        end

        local slp = {x = slx, y = sly, z = slz}
        local elp = {x = elx, y = ely, z = elz}

        ic(cam, sp, ep, slp, elp, 2000, i)
    end

    if fo and ca then
        natives.cam_doScreenFadeOut(fot)
        system.yield(fot)
    end
    kill_camera()
end

system.registerDestructor(function()
    if rm then
        rm = false
        system.unregisterTick()
    end
    kill_camera()
end)

function kill_camera()
    if cb then natives.cam_requestLetterBoxNow(false, false) end
    if cam then
        natives.cam_renderScriptCams(false, true, 2000, false, false, 0)
        natives.cam_destroyCam(cam, false)
        natives.cam_doScreenFadeOut(0)
        natives.cam_doScreenFadeIn(1000)
        menu.setToggleValue('self', '~t6~Move Camera', false)
        cam = nil
    end
    ca = false
end

-- Config
local function serialize_nodes(mv_nodes, lk_nodes)
    local parts = {}
    table.insert(parts, "OC:")
    for _, node in ipairs(mv_nodes) do
        table.insert(parts, string.format("%f,%f,%f", node.x, node.y, node.z))
    end
    table.insert(parts, "SEPARATOR")
    table.insert(parts, "SLC:")
    for _, node in ipairs(lk_nodes) do
        table.insert(parts, string.format("%f,%f,%f", node.x, node.y, node.z))
    end
    return table.concat(parts, ";")
end

local function deserialize_nodes(data_str)
    local mv_nodes = {}
    local lk_nodes = {}
    if data_str == "" then return mv_nodes, lk_nodes end

    local tokens = {}
    for seg in string.gmatch(data_str, "([^;]+)") do
        table.insert(tokens, seg)
    end

    local mode = "OC"
    for _, t in ipairs(tokens) do
        if t == "SEPARATOR" then
            mode = "SLC"
        elseif t == "OC:" or t == "SLC:" then
            -- skip
        else
            local x, y, z = t:match("([^,]+),([^,]+),([^,]+)")
            if x and y and z then
                local fx = tonumber(x)
                local fy = tonumber(y)
                local fz = tonumber(z)
                if mode == "OC" then
                    table.insert(mv_nodes, {x = fx, y = fy, z = fz})
                else
                    table.insert(lk_nodes, {x = fx, y = fy, z = fz})
                end
            end
        end
    end

    return mv_nodes, lk_nodes
end

local function save_data_set(set_name, mv_nodes, lk_nodes)
    local all_sets = config.loadString("camera_datasets", "")
    local set_list = {}
    for seg in string.gmatch(all_sets, "([^;]+)") do
        table.insert(set_list, seg)
    end

    local found = false
    for _, v in ipairs(set_list) do
        if v == set_name then
            found = true
            break
        end
    end

    if not found then
        table.insert(set_list, set_name)
        config.saveString("camera_datasets", table.concat(set_list, ";"))
    end

    local serialized = serialize_nodes(mv_nodes, lk_nodes)
    config.saveString("camdata_" .. set_name, serialized)
end

local function load_data_set(set_name)
    local saved_str = config.loadString("camdata_" .. set_name, "")
    local new_mv, new_lk = deserialize_nodes(saved_str)
    oc = new_mv
    slc = new_lk
    rc()
end

local function delete_data_set(set_name)
    local all_sets = config.loadString("camera_datasets", "")
    if all_sets == "" then
        return
    end

    local set_list = {}
    for seg in string.gmatch(all_sets, "([^;]+)") do
        table.insert(set_list, seg)
    end

    local new_list = {}
    for _, v in ipairs(set_list) do
        if v ~= set_name then
            table.insert(new_list, v)
        end
    end

    config.saveString("camera_datasets", table.concat(new_list, ";"))

    config.saveString("camdata_" .. set_name, "")

    notifications.alertDanger("Camera Mod", "Config " .. set_name .. " Deleted")
end

local function load_buttons()
    local all_sets = config.loadString("camera_datasets", "")
    if all_sets == "" then
        return
    end

    local cfg_submenu = menu.addSubmenu('self', 'Configs', '')
    for seg in string.gmatch(all_sets, "([^;]+)") do
        local nm = seg
        menu.addButton(cfg_submenu, "" .. nm, "To see new cfgs reload the script.", function()
            load_data_set(nm)
        end)
    end

    menu.addButton(cfg_submenu, "~e~Delete Cfg", "Type the name of the cfg you want to delete then reload the script. (Can't delete the first cfg made)", function()
        keyboard.getInput("", function(user_input)
            if user_input and user_input ~= "" then
                delete_data_set(user_input)
            end
        end)
    end)
end

load_buttons()

menu.addButton('self', 'Save Cfg', 'Saves all current node positions.', function()
    keyboard.getInput("", function(user_input)
        if user_input and user_input ~= "" then
            save_data_set(user_input, oc, slc)
        end
    end)
end)

-- Main
menu.addDivider('self', 'Camera Mod')
menu.addButton('self', '~q~Add Move Node', '', function()
    local x, y, z = player.getLocalPedCoords()
    table.insert(oc, {x = x, y = y, z = z})
    rc()
end)

menu.addButton('self', '~q~Add Look Node', '', function()
    local x, y, z = player.getLocalPedCoords()
    table.insert(slc, {x = x, y = y, z = z})
end)

menu.addToggleButton('self', '~pa~Render Nodes', '', false, function(t)
    rm = t
    if rm then
        system.registerTick(function()
            if not rm then
                system.unregisterTick()
                return
            end
            for i, c in ipairs(sc) do
                local r, g, b = 255, 255, 255
                if i == 1 then
                    r, g, b = 0, 255, 0
                elseif i == #sc then
                    r, g, b = 255, 0, 0
                end
                if si > 0 and si <= #sc and i == si then
                    r, g, b = 255, 0, 255
                end
                natives.graphics_drawMarker(0x50638AB9, c.x, c.y, c.z, 0, 0, 0, 0, 0, 0, 0.3, 0.3, 0.3, r, g, b, 100, false, false, 0, true, '', '', false)
            end
            for i, c in ipairs(slc) do
                local r, g, b = 255, 255, 0
                if i == 1 then
                    r, g, b = 0, 0, 255
                elseif i == #slc then
                    r, g, b = 255, 165, 0
                end
                if si > #sc and si - #sc == i then
                    r, g, b = 255, 0, 255
                end
                natives.graphics_drawMarker(0x50638AB9, c.x, c.y, c.z, 0, 0, 0, 0, 0, 0, 0.3, 0.3, 0.3, r, g, b, 100, false, false, 0, true, '', '', false)
            end
        end)
    else
        system.unregisterTick()
    end
end)

menu.addToggleButton('self', '~t6~Move Camera', '', false, function(t)
    if t and #sc >= 2 then
        ca = true
        move_camera()
    else
        kill_camera()
    end
end)

menu.addButton('self', '~t4~Clear Move Nodes', '', function()
    oc = {}
    sc = {}
    si = 0
    lx = 0.0
    ly = 0.0
    lz = 0.0
end)

menu.addButton('self', '~t4~Clear Look Nodes', '', function()
    slc = {}
    si = 0
    lx = 0.0
    ly = 0.0
    lz = 0.0
end)

menu.addButton('self', '~e~Reset Cam', '', function()
    natives.cam_renderScriptCams(false, true, 2000, false, false, 0)
    natives.cam_destroyAllCams(true)
    natives.cam_doScreenFadeOut(0)
    natives.cam_doScreenFadeIn(1000)
end)

-- Visual
menu.addDivider('self', 'Visuals')
menu.addToggleButton('self', 'Fade In', '', false, function(t)
    fi = t
end)
menu.addIntSpinner('self', 'Fade In Time (ms)', '', 0, 10000, 500, 1000, function(v)
    fit = v
end)
menu.addToggleButton('self', 'Fade Out', '', false, function(t)
    fo = t
end)
menu.addIntSpinner('self', 'Fade Out Time (ms)', '', 0, 10000, 500, 1000, function(v)
    fot = v
end)
menu.addFloatSpinner('self', 'Camera FOV', '', 1.0, 130.0, 1.0, 50.0, function(v)
    fov = v
end)
menu.addFloatSpinner('self', 'Camera Focus Distance', '', 1.0, 200.0, 1.0, 50.0, function(v)
    fd = v
end)
menu.addToggleButton('self', 'Cinematic Bars', '', false, function(t)
    cb = t
end)

-- Advanced
menu.addDivider('self', 'Advanced')
menu.addToggleButton('self', 'Follow Player', '', false, function(t)
    fp = t
end)
menu.addToggleButton('self', 'Manual Speed', '', false, function(t)
    ms = t
end)
menu.addFloatSpinner('self', 'Cam Speed', '', 0.1, 1000.0, 0.1, 10.0, function(v)
    ms_val = v
end)
menu.addIntSpinner('self', 'Extra Nodes', '', 0, 100, 1, 0, function(v)
    eps = v
    rc()
end)

menu.addButton('self', 'Select Node', 'Will skip extra nodes', function()
    local total = #sc + #slc
    if total == 0 then
        si = 0
        return
    end
    local function se()
        while si <= #sc and not gi(si) do
            si = si + 1
        end
    end
    si = si + 1
    if si > total then si = 1 end
    if si <= #sc then
        se()
        if si > total then si = 1 end
    end
    if si > total then si = 1 end
    lx = 0.0
    ly = 0.0
    lz = 0.0
end)

menu.addButton('self', 'Remove Node', '', function()
    local total = #sc + #slc
    if si > 0 and si <= total then
        if si <= #sc then
            if gi(si) then
                local N = #oc
                local s = 1 + eps
                local tt = (N - 1) * s + 1
                if si == tt then
                    table.remove(oc, N)
                    rc()
                else
                    local seg = (si - 1) // s + 1
                    table.remove(oc, seg)
                    rc()
                end
            end
        else
            local ln_i = si - #sc
            if ln_i >= 1 and ln_i <= #slc then
                table.remove(slc, ln_i)
            end
        end
    end
    si = 0
    lx = 0.0
    ly = 0.0
    lz = 0.0
end)

menu.addFloatSpinner('self', 'Increment Value', 'How fast nodes move', 0.1, 100.0, 0.1, 1.0, function(val)
    iv = val
end)

menu.addFloatSpinner('self', 'Selected Node X', '', -10000.0, 10000.0, 1.0, 0.0, function(val)
    local total = #sc + #slc
    if si > 0 and si <= total then
        local delta = (val - lx) * iv
        lx = val
        if si <= #sc then
            if gi(si) then
                local N = #oc
                local s = 1 + eps
                local tt = (N - 1) * s + 1
                if si == tt then
                    oc[N].x = oc[N].x + delta
                else
                    local seg = (si - 1) // s + 1
                    oc[seg].x = oc[seg].x + delta
                end
                rc()
            end
        else
            local ln_i = si - #sc
            if ln_i >= 1 and ln_i <= #slc then
                slc[ln_i].x = slc[ln_i].x + delta
            end
        end
    end
end)

menu.addFloatSpinner('self', 'Selected Node Y', '', -10000.0, 10000.0, 1.0, 0.0, function(val)
    local total = #sc + #slc
    if si > 0 and si <= total then
        local delta = (val - ly) * iv
        ly = val
        if si <= #sc then
            if gi(si) then
                local N = #oc
                local s = 1 + eps
                local tt = (N - 1) * s + 1
                if si == tt then
                    oc[N].y = oc[N].y + delta
                else
                    local seg = (si - 1) // s + 1
                    oc[seg].y = oc[seg].y + delta
                end
                rc()
            end
        else
            local ln_i = si - #sc
            if ln_i >= 1 and ln_i <= #slc then
                slc[ln_i].y = slc[ln_i].y + delta
            end
        end
    end
end)

menu.addFloatSpinner('self', 'Selected Node Z', '', -10000.0, 10000.0, 1.0, 0.0, function(val)
    local total = #sc + #slc
    if si > 0 and si <= total then
        local delta = (val - lz) * iv
        lz = val
        if si <= #sc then
            if gi(si) then
                local N = #oc
                local s = 1 + eps
                local tt = (N - 1) * s + 1
                if si == tt then
                    oc[N].z = oc[N].z + delta
                else
                    local seg = (si - 1) // s + 1
                    oc[seg].z = oc[seg].z + delta
                end
                rc()
            end
        else
            local ln_i = si - #sc
            if ln_i >= 1 and ln_i <= #slc then
                slc[ln_i].z = slc[ln_i].z + delta
            end
        end
    end
end)