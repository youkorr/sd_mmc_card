import esphome.codegen as cg
import esphome.config_validation as cv

CONF_ID = 'id'  # Add this line to define CONF_ID
CONF_SERVER = 'server'
CONF_USERNAME = 'username'
CONF_PASSWORD = 'password'
CONF_REMOTE_PATHS = 'remote_paths'
CONF_LOCAL_PORT = 'local_port'

DEPENDENCIES = []
AUTO_LOAD = []

ftp_http_proxy_ns = cg.esphome_ns.namespace('ftp_http_proxy')
FTPHTTPProxy = ftp_http_proxy_ns.class_('FTPHTTPProxy', cg.Component)

def validate_remote_paths(value):
    # Vérification personnalisée pour les chemins distants
    if not isinstance(value, list):
        raise cv.Invalid("Remote paths must be a list of strings")
    return [cv.string(path) for path in value]

CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.declare_id(FTPHTTPProxy),  # Declare the ID for the component
    cv.Required(CONF_SERVER): cv.string,
    cv.Required(CONF_USERNAME): cv.string,
    cv.Required(CONF_PASSWORD): cv.string,
    cv.Required(CONF_REMOTE_PATHS): validate_remote_paths,
    cv.Optional(CONF_LOCAL_PORT, default=8000): cv.port,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    
    # Configuration des paramètres
    cg.add(var.set_ftp_server(config[CONF_SERVER]))
    cg.add(var.set_username(config[CONF_USERNAME]))
    cg.add(var.set_password(config[CONF_PASSWORD]))
    
    # Ajout des chemins distants
    for remote_path in config[CONF_REMOTE_PATHS]:
        cg.add(var.add_remote_path(remote_path))
    
    cg.add(var.set_local_port(config[CONF_LOCAL_PORT]))
