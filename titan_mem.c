/**
 *
 * Flash memory interface Library
 * TITANLAB INC 2019
 * Keith Wakeham
 * 
 * 
 */

#include "titan_mem.h"
#include "nrf_fstorage_sd.h"
// #include "nrf_fstorage.h" //no soft device
#include "nrf_log.h"
#include "nrf_sdh_ble.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "fds.h"
#include "app_scheduler.h"
#include "nrf_pwr_mgmt.h"

#define MEM_START 0x70000
#define MEM_END 0x71fff
#define FILE_ID         0x0001  /* The ID of the file to write the records into. */
#define RECORD_KEY_1    0x1234  /* A key for the first record. */

#define FILE_ID_SLEEP         0x0002  /* The ID of the file to write the records into. */
#define RECORD_KEY_SLEEP_1    0x0ABC  /* A key for the first record. */

static uint32_t   const m_deadbeef = 0xDEADBEEF;
// static char       const m_hello[]  = "Hello, world!";
fds_record_t        record;
fds_record_desc_t   record_desc;

// static nrf_fstorage_api_t * p_fs_api;
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

bool gc_flag = false; //garbage collection flag


/* Flag to check fds initialization. */
static bool volatile m_fds_initialized2 = false;

char const * fds_err_str[] =
{
    "NRF_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

/* Configuration data. */
static epx_configuration_t m_epx_cfg =
{
    .num_gears = 0,
    .gear_pos = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},

    .Kp = 0.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,

    // .current_rotations = 0,
    // .current_angle = 0,
    // .current_gear = 0,

    // .upshifts = 0,
    // .downshifts = 0,

    .sin_min = 0,
    .sin_max = 0,
    .cos_min = 0,
    .cos_max = 0,
};

static epx_position_configuration_t m_epx_position_cfg =
{
    .current_rotations = 0,
    .current_angle = 0,
    .current_gear = 0,

    .upshifts = 0,
    .downshifts = 0,
};

/* A record containing configuration data. */
static fds_record_t const m_epx_record =
{
    .file_id           = FILE_ID,
    .key               = RECORD_KEY_1,
    .data.p_data       = &m_epx_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_epx_cfg)+3) / sizeof(uint32_t), //+3 for rounding
};

/* A record containing configuration data. */
static fds_record_t const m_epx_position_record =
{
    .file_id           = FILE_ID_SLEEP,
    .key               = RECORD_KEY_SLEEP_1,
    .data.p_data       = &m_epx_position_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_epx_position_cfg)+3) / sizeof(uint32_t), //+3 for rounding
};


NRF_FSTORAGE_DEF(nrf_fstorage_t titan_mem) =
{
    .evt_handler = fstorage_evt_handler,
    .start_addr = MEM_START,
    .end_addr   = MEM_END,
};


static void tm_fds_evt_handler(fds_evt_t const * p_evt)
{
    NRF_LOG_INFO("Event: %s received (%s)",
                  fds_evt_str[p_evt->id],
                  fds_err_str[p_evt->result]);

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized2 = true;
                NRF_LOG_INFO("FDS initialized");
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        case FDS_EVT_GC:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                gc_flag = false;
                NRF_LOG_INFO("GC successful");
            }
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}

static void wait_for_fds_ready(void)
{
    while(!m_fds_initialized2)
    {
        app_sched_execute();
        nrf_pwr_mgmt_run();
    }
}

void tm_fds_init()
{
    ret_code_t ret = fds_register(tm_fds_evt_handler);
    if (ret != NRF_SUCCESS)
    {
        // Registering of the FDS event handler has failed.
    }
    NRF_LOG_INFO("MEM: registered event handler");

    ret = fds_init();
    if (ret != NRF_SUCCESS)
    {
        // Handle error.
    }
    NRF_LOG_INFO("MEM: fds initialize");

    wait_for_fds_ready();

}

void tm_fds_test_write()
{
    record.file_id           = FILE_ID;
    record.key               = RECORD_KEY_1;
    record.data.p_data       = &m_deadbeef;
    record.data.length_words = 1;   /* one word is four bytes. */
    ret_code_t rc;
    rc = fds_record_write(&record_desc, &record);
    if (rc != NRF_SUCCESS)
    {
        /* Handle error. */
    }
}

epx_configuration_t tm_fds_epx_config (void)
{
    return m_epx_cfg;
}

epx_position_configuration_t tm_fds_epx_position (void)
{
    return m_epx_position_cfg;
}

void tm_fds_test_retrieve()
{
    // ret_code_t rc;
    fds_flash_record_t  flash_record;
    fds_record_desc_t   record_desc;
    fds_find_token_t    ftok;
    uint32_t test_data;
    /* It is required to zero the token before first use. */
    memset(&ftok, 0x00, sizeof(fds_find_token_t));
    /* Loop until all records with the given key and file ID have been found. */
    while (fds_record_find(FILE_ID, RECORD_KEY_1, &record_desc, &ftok) == NRF_SUCCESS)
    {
        if (fds_record_open(&record_desc, &flash_record) != NRF_SUCCESS)
        {
            /* Handle error. */
        }
        /* Access the record through the flash_record structure. */
        memcpy(&test_data, flash_record.p_data, sizeof(epx_configuration_t));

        NRF_LOG_INFO("The record is: %d",test_data);
        /* Close the record when done. */
        if (fds_record_close(&record_desc) != NRF_SUCCESS)
        {
            /* Handle error. */
        }
    }
    
}

void tm_fds_test_delete()
{
    ret_code_t ret = fds_record_delete(&record_desc);
    if (ret != NRF_SUCCESS)
    {
        /* Error. */
    }
}

void tm_fds_config_init()
{
    ret_code_t rc;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    fds_stat_t stat = {0};
    rc = fds_stat(&stat);
    APP_ERROR_CHECK(rc);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    tm_fds_gc();

    rc = fds_record_find(FILE_ID, RECORD_KEY_1, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Found config file...");
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        rc = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(rc);

        /* Copy the configuration from flash into m_epx_cfg. */
        memcpy(&m_epx_cfg, config.p_data, sizeof(epx_configuration_t));

        /* Close the record when done reading. */
        rc = fds_record_close(&desc);
        APP_ERROR_CHECK(rc);
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing config file...");

        rc = fds_record_write(&desc, &m_epx_record);
        APP_ERROR_CHECK(rc);
    }

    rc = fds_record_find(FILE_ID_SLEEP, RECORD_KEY_SLEEP_1, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Found sleep file...");
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        rc = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(rc);

        /* Copy the configuration from flash into m_epx_cfg. */
        memcpy(&m_epx_position_cfg, config.p_data, sizeof(epx_position_configuration_t));

        /* Close the record when done reading. */
        rc = fds_record_close(&desc);
        APP_ERROR_CHECK(rc);
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing sleep file...");

        rc = fds_record_write(&desc, &m_epx_position_record);
        APP_ERROR_CHECK(rc);
    }
}

void tm_fds_gc()
{
    ret_code_t rc;
    fds_stat_t stat = {0};
    rc = fds_stat(&stat);
    APP_ERROR_CHECK(rc);

    if (!gc_flag)
    {
        if(stat.dirty_records > 60)
        {
            gc_flag = true;
            fds_gc();
            NRF_LOG_INFO("Garbage Collecting");
        }
        else
        {
            gc_flag = false;
        }
        
    }
}

void tm_fds_config_update()
{
    ret_code_t rc;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    tm_fds_gc(); //run garbage collection

    rc = fds_record_find(FILE_ID, RECORD_KEY_1, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
        //debug info
        // char tm_debug_message[20];
        // sprintf(tm_debug_message,"%ld, %.5f \n",m_epx_cfg.zero, m_epx_cfg.calibration);
        // NRF_LOG_INFO("%s",tm_debug_message);

        rc = fds_record_update(&desc, &m_epx_record);
        APP_ERROR_CHECK(rc);
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing config file...");
        rc = fds_record_write(&desc, &m_epx_record);
        APP_ERROR_CHECK(rc);
    }
}

void tm_fds_position_update()
{
    ret_code_t rc;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    tm_fds_gc(); //run garbage collection

    rc = fds_record_find(FILE_ID_SLEEP, RECORD_KEY_SLEEP_1, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
        //debug info
        // char tm_debug_message[20];
        // sprintf(tm_debug_message,"%ld, %.5f \n",m_epx_cfg.zero, m_epx_cfg.calibration);
        // NRF_LOG_INFO("%s",tm_debug_message);

        rc = fds_record_update(&desc, &m_epx_position_record);
        APP_ERROR_CHECK(rc);
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing position file...");
        rc = fds_record_write(&desc, &m_epx_position_record);
        APP_ERROR_CHECK(rc);
    }
}


void storage_init()
{
    ret_code_t rc;

    rc = nrf_fstorage_init(
        &titan_mem,       /* You fstorage instance, previously defined. */
        &nrf_fstorage_sd,   /* Name of the backend. */
        NULL                /* Optional parameter, backend-dependant. */
    );

    APP_ERROR_CHECK(rc);
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

void mem_epx_update(epx_configuration_t config_towrite)
{
    m_epx_cfg = config_towrite;
    tm_fds_config_update();
}

void mem_epx_position_update(epx_position_configuration_t position_config_towrite)
{
    m_epx_position_cfg = position_config_towrite;
    tm_fds_position_update();
}