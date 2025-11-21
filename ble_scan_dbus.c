#include <gio/gio.h>
#include <glib.h>
#include <stdio.h>

// デバイスを表示するコールバック
static void interfaces_added(GDBusConnection *conn,
                             const gchar *sender_name,
                             const gchar *object_path,
                             const gchar *interface_name,
                             const gchar *signal_name,
                             GVariant *parameters,
                             gpointer user_data) {

    const gchar *object;
    GVariant *interfaces;
    g_variant_get(parameters, "(&o@a{sa{sv}})", &object, &interfaces);

    GVariantIter *iter;
    const gchar *iface_name;
    GVariant *props;

    g_variant_get(interfaces, "a{sa{sv}}", &iter);
    while (g_variant_iter_next(iter, "{&s@a{sv}}", &iface_name, &props)) {
        if (g_strcmp0(iface_name, "org.bluez.Device1") == 0) {
            GVariant *addr_var = g_variant_lookup_value(props, "Address", NULL);
            GVariant *name_var = g_variant_lookup_value(props, "Name", NULL);
            GVariant *rssi_var = g_variant_lookup_value(props, "RSSI", NULL);

            const char *addr = addr_var ? g_variant_get_string(addr_var, NULL) : "(unknown)";
            const char *name = name_var ? g_variant_get_string(name_var, NULL) : "(unknown)";
            int rssi = 0;
	    if (rssi_var)
    	    rssi = g_variant_get_int16(rssi_var); // <- BYTE ではなく int16

            printf("Found device: %s  Name: %s  RSSI: %d dBm\n", addr, name, rssi);

            if (addr_var) g_variant_unref(addr_var);
            if (name_var) g_variant_unref(name_var);
            if (rssi_var) g_variant_unref(rssi_var);
        }
        g_variant_unref(props);
    }
    g_variant_iter_free(iter);
    g_variant_unref(interfaces);
}

// 既存デバイスをすべて削除
static void remove_all_devices(GDBusConnection *conn) {
    GError *error = NULL;

    // org.freedesktop.DBus.ObjectManager の GetManagedObjects で全オブジェクトを取得
    GVariant *objects = g_dbus_connection_call_sync(
        conn,
        "org.bluez",
        "/",
        "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects",
        NULL,
        G_VARIANT_TYPE("(a{oa{sa{sv}}})"),
        G_DBUS_CALL_FLAGS_NONE,
        -1,
        NULL,
        &error
    );

    if (!objects) {
        fprintf(stderr, "Failed to get managed objects: %s\n", error->message);
        return;
    }

    GVariantIter *obj_iter;
    g_variant_get(objects, "(a{oa{sa{sv}}})", &obj_iter);

    const gchar *obj_path;
    GVariant *interfaces;

    while (g_variant_iter_next(obj_iter, "{&o@a{sa{sv}}}", &obj_path, &interfaces)) {
        GVariantIter *iface_iter;
        const gchar *iface_name;
        GVariant *props;

        g_variant_get(interfaces, "a{sa{sv}}", &iface_iter);
        while (g_variant_iter_next(iface_iter, "{&s@a{sv}}", &iface_name, &props)) {
            if (g_strcmp0(iface_name, "org.bluez.Device1") == 0) {
                // デバイス削除
                g_dbus_connection_call_sync(conn,
                                            "org.bluez",
                                            "/org/bluez/hci0",
                                            "org.bluez.Adapter1",
                                            "RemoveDevice",
                                            g_variant_new("(o)", obj_path),
                                            NULL,
                                            G_DBUS_CALL_FLAGS_NONE,
                                            -1,
                                            NULL,
                                            &error);
            }
            g_variant_unref(props);
        }
        g_variant_iter_free(iface_iter);
        g_variant_unref(interfaces);
    }
    g_variant_iter_free(obj_iter);
    g_variant_unref(objects);
}

int main(int argc, char *argv[]) {
    GError *error = NULL;
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);

    GDBusConnection *conn = g_bus_get_sync(G_BUS_TYPE_SYSTEM, NULL, &error);
    if (!conn) {
        fprintf(stderr, "Failed to get system bus: %s\n", error->message);
        return 1;
    }

    // 既存デバイスを削除
    remove_all_devices(conn);

    // InterfacesAdded シグナル登録
    g_dbus_connection_signal_subscribe(conn,
                                       "org.bluez",
                                       "org.freedesktop.DBus.ObjectManager",
                                       "InterfacesAdded",
                                       NULL,
                                       NULL,
                                       G_DBUS_SIGNAL_FLAGS_NONE,
                                       interfaces_added,
                                       NULL,
                                       NULL);

    // スキャン開始
    GVariant *result = g_dbus_connection_call_sync(
        conn,
        "org.bluez",
        "/org/bluez/hci0",
        "org.bluez.Adapter1",
        "StartDiscovery",
        NULL,
        NULL,
        G_DBUS_CALL_FLAGS_NONE,
        -1,
        NULL,
        &error
    );

    if (!result) {
        fprintf(stderr, "Failed to start discovery: %s\n", error->message);
        return 1;
    } else {
        g_variant_unref(result);
    }

    printf("Scanning for BLE devices...\n");
    g_main_loop_run(loop);

    return 0;
}
